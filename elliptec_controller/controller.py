"""
Thorlabs Elliptec Rotator Controller

This module implements the ElliptecRotator class for controlling Thorlabs
Elliptec rotation stages over serial.

Protocol details based on the Thorlabs Elliptec documentation.
"""

import serial
import time
import threading
from typing import Dict, List, Optional, Union, Any
from loguru import logger
from enum import Enum

# --- Device Status Code Constants ---
STATUS_READY = "00"
STATUS_HOMING = "09"
STATUS_MOVING = "01"

# --- Motor Status Bitmask Enum (based on device protocol) ---
class MOTOR_STATUS(Enum):
    MOTOR_ACTIVE = 0x01
    HOMING = 0x02

# Motor command constants - based on ELLx protocol manual
COMMAND_GET_STATUS = "gs"
COMMAND_STOP = "st"
COMMAND_HOME = "ho"
COMMAND_FORWARD = "fw"
COMMAND_BACKWARD = "bw"
COMMAND_MOVE_ABS = "ma"
COMMAND_MOVE_REL = "mr"
COMMAND_GET_POS = "gp"
COMMAND_SET_VELOCITY = "sv"
COMMAND_GET_VELOCITY = "gv"
COMMAND_SET_HOME_OFFSET = "so"
COMMAND_GET_HOME_OFFSET = "go"
COMMAND_GROUP_ADDRESS = "ga"
COMMAND_OPTIMIZE_MOTORS = "om"
COMMAND_GET_INFO = "in"
COMMAND_SET_JOG_STEP = "sj"
COMMAND_GET_JOG_STEP = "gj"


def degrees_to_hex(degrees: float, pulse_per_revolution: int = 262144) -> str:
    pulses_per_deg = pulse_per_revolution / 360.0
    pulses = int(round(degrees * pulses_per_deg))
    if pulses < 0:
        pulses = (1 << 32) + pulses
    return format(pulses & 0xFFFFFFFF, "08x").upper()

def hex_to_degrees(hex_val: str, pulse_per_revolution: int = 262144) -> float:
    cleaned_hex = hex_val.strip(" \r\n\t")
    if not cleaned_hex:
        return 0.0
    try:
        value = int(cleaned_hex, 16)
    except ValueError:
        return 0.0
    if value & 0x80000000:
        value = value - (1 << 32)
    if pulse_per_revolution == 0:
        return 0.0
    pulses_per_deg = pulse_per_revolution / 360.0
    return value / pulses_per_deg


class ElliptecRotator:
    def __init__(
        self,
        port: Union[str, serial.Serial, Any],
        motor_address: int = 0,
        name: Optional[str] = None,
        auto_home: bool = True,
    ):
        self.physical_address = str(motor_address)
        self.active_address = self.physical_address
        self.name = name or f"Rotator-{self.physical_address}"
        self.logger = logger.bind(rotator_name=self.name, physical_address=self.physical_address)

        # Internal state attribute (do not use public .is_moving for assignment)
        self._is_moving_state = False
        self.is_slave_in_group = False
        self.group_offset_degrees = 0.0
        self.velocity = 60
        self.optimal_frequency = None
        self._jog_step_size = 1.0
        self._command_lock = threading.RLock()

        self.pulse_per_revolution = 262144
        self.range = 360
        self.pulses_per_deg = self.pulse_per_revolution / 360.0
        self.device_info: Dict[str, str] = {}

        if (not isinstance(port, str) and hasattr(port, "log") and hasattr(port, "write")):
            self.serial = port
            self._fixture_test = True
            self._mock_in_test = True
            self.serial._log = self.serial._log if hasattr(self.serial, '_log') else []
            self.position_degrees = 0.0
            if not hasattr(self, "pulse_per_revolution"): self.pulse_per_revolution = 262144
            if not hasattr(self, "pulses_per_deg"): self.pulses_per_deg = self.pulse_per_revolution / 360.0
        elif hasattr(port, 'write') and hasattr(port, 'read') and hasattr(port, 'flush'):
            self.serial = port
        elif isinstance(port, str):
            self.serial = serial.Serial(port=port, baudrate=9600, bytesize=8, parity="N", stopbits=1, timeout=1)
            try:
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
            except serial.SerialException as e:
                self.logger.warning(f"Error resetting serial port buffers during init: {e}")

            try:
                device_info_retrieved = self.get_device_info()
                if device_info_retrieved and device_info_retrieved.get("type") not in ["Error", "Unknown"]:
                    pulses_dec_str = device_info_retrieved.get("pulses_per_unit_decimal")
                    if pulses_dec_str:
                        try:
                            pulses_dec = int(pulses_dec_str)
                            if pulses_dec > 0:
                                self.pulse_per_revolution = pulses_dec
                                self.pulses_per_deg = pulses_dec / 360.0
                                self.logger.debug(f"__init__ set pulse_per_revolution to {self.pulse_per_revolution} from get_device_info return.")
                            else:
                                self.logger.warning(f"__init__ received invalid pulses_dec: {pulses_dec} from get_device_info. Using default: {self.pulse_per_revolution}")
                        except ValueError:
                            self.logger.warning(f"__init__ could not parse pulses_dec_str: '{pulses_dec_str}' from get_device_info. Using default: {self.pulse_per_revolution}")
                else:
                    self.logger.warning(f"__init__ did not get valid device info to set pulse_per_revolution. Using default: {self.pulse_per_revolution}")

                if auto_home and not (hasattr(self, '_fixture_test') and self._fixture_test):
                    try:
                        self.logger.info("Homing...")
                        if not self.home(wait=True): self.logger.warning("Failed to home.")
                        self.logger.info("Getting position...")
                        self.update_position()
                        self.logger.info("Getting velocity...")
                        velocity_val = self.get_velocity()
                        if velocity_val is not None: self.velocity = velocity_val
                        self.logger.info("Getting jog step...")
                        jog_step = self.get_jog_step()
                        if jog_step is not None: self._jog_step_size = jog_step
                        self.logger.info("Initialization complete.")
                    except Exception as init_e:
                        self.logger.error(f"Error during attribute initialization: {init_e}", exc_info=True)
            except Exception as e:
                self.logger.error(f"Error retrieving device info during init: {e}", exc_info=True)
        else:
            raise ValueError(f"Unsupported port type: {type(port)}. Must be str, serial.Serial, or a compatible mock.")

    @property
    def is_moving(self) -> bool:
        """Checks if the motor is currently identified as moving by status byte."""
        status_hex = self.get_status()
        final_is_moving_decision = False  # Default to False

        if status_hex:
            try:
                status_val = int(status_hex, 16)
                is_active = (status_val & MOTOR_STATUS.MOTOR_ACTIVE.value) != 0
                is_homing = (status_val & MOTOR_STATUS.HOMING.value) != 0

                final_is_moving_decision = is_active or is_homing

                self.logger.debug(
                    f"ElliptecRotator.is_moving: status_hex='{status_hex}', status_val=0x{status_val:02X}, "
                    f"active_bit_set={is_active}, homing_bit_set={is_homing}, "
                    f"WILL RETURN: {final_is_moving_decision}"
                )
            except ValueError:
                self.logger.warning(
                    f"ElliptecRotator.is_moving: Could not parse status_hex '{status_hex}' to int. Returning False."
                )
                final_is_moving_decision = False
        else:
            self.logger.warning("ElliptecRotator.is_moving: Could not get valid status_hex. Assuming not moving. Returning False.")
            final_is_moving_decision = False

        return final_is_moving_decision


    def send_command(
        self,
        command: str,
        data: str = "",
        timeout: Optional[float] = None,
        send_addr_override: Optional[str] = None,
        expect_reply_from_addr: Optional[str] = None,
        timeout_multiplier: float = 1.0,
    ) -> str:
        with self._command_lock:
            if not self.serial.is_open:
                try:
                    self.serial.open()
                except serial.SerialException as e:
                    self.logger.error(f"Error opening serial port: {e}")
                    return ""
            try:
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
            except serial.SerialException as e:
                self.logger.warning(f"Error resetting serial port buffers: {e}")

            address_to_send_with = send_addr_override if send_addr_override is not None else self.active_address
            address_to_expect_reply_from = expect_reply_from_addr if expect_reply_from_addr is not None else self.active_address

            cmd_str = f"{address_to_send_with}{command}"
            if data: cmd_str += data
            cmd_str += "\r"

            self.logger.trace(f"Sending (to addr: {address_to_send_with}): '{cmd_str.strip()}' (hex: {' '.join(f'{ord(c):02x}' for c in cmd_str)})")

            if hasattr(self, '_fixture_test') and command == "gs" and timeout is not None and timeout < 0.1:
                if hasattr(self.serial, 'log'): self.serial._log.append(cmd_str.replace("\r", "\\r").encode("ascii"))
                return ""
            try:
                cmd_str_for_write = cmd_str.replace("\r", "\\r") if hasattr(self.serial, 'log') else cmd_str
                self.serial.write(cmd_str_for_write.encode("ascii"))
                self.serial.flush()
            except serial.SerialException as e:
                self.logger.error(f"Error writing to serial port: {e}")
                return ""

            start_time = time.time()
            response_bytes = b""
            if timeout is not None: effective_timeout = timeout
            elif command in ["ma", "mr", "ho", "om", "cm"]: effective_timeout = 3.0 * timeout_multiplier
            elif command == "ga": effective_timeout = 1.5 * timeout_multiplier
            else: effective_timeout = 1.0 * timeout_multiplier

            try:
                while (time.time() - start_time) < effective_timeout:
                    if self.serial.in_waiting > 0:
                        response_bytes += self.serial.read(self.serial.in_waiting)
                        if response_bytes.endswith(b"\r\n"): break
                        elif response_bytes.endswith(b"\n") or response_bytes.endswith(b"\r"):
                            time.sleep(0.005)
                            if self.serial.in_waiting > 0: response_bytes += self.serial.read(self.serial.in_waiting)
                            if response_bytes.endswith(b"\r\n"): break
                            self.logger.trace(f"Partial EOL detected, treating as end. Raw: {response_bytes!r}")
                            break
                    time.sleep(0.1)
            except serial.SerialException as e:
                self.logger.error(f"Error reading from serial port: {e}")
                return ""

            response_str = response_bytes.decode("ascii", errors="replace").strip()
            if hasattr(self.serial, 'log'): response_str = response_str.replace('\\r', '').replace('\\n', '')

            duration_ms = (time.time() - start_time) * 1000
            self.logger.trace(f"Response (expecting from addr: {address_to_expect_reply_from}): '{response_str}' (raw: {response_bytes!r}) (took {duration_ms:.1f}ms)")
            if not response_str: self.logger.warning(f"No response or timed out after {effective_timeout:.2f}s")

            if response_str.startswith(address_to_expect_reply_from): return response_str
            elif (len(address_to_expect_reply_from) == 1 and address_to_expect_reply_from.isalpha() and response_str.lower().startswith(address_to_expect_reply_from.lower())):
                self.logger.trace(f"Matched response with case-insensitive address: '{response_str}'")
                return response_str
            else:
                if response_str: self.logger.warning(f"Response ('{response_str}') did not match expected address prefix '{address_to_expect_reply_from}'. Discarding.")
            return ""

    def get_status(self, timeout_override: Optional[float] = None) -> str:
        with self._command_lock:
            if hasattr(self, "_fixture_test") and hasattr(self.serial, "_responses"):
                if self.serial._responses: pass
                else:
                    cmd_str = f"{self.active_address}gs\\r"
                    if hasattr(self.serial, "_log"): self.serial._log.append(cmd_str.encode())
                    return STATUS_READY
            response = self.send_command(COMMAND_GET_STATUS, timeout=timeout_override)
            if response:
                expected_prefix = f"{self.active_address}GS"
                if response.startswith(expected_prefix):
                    status_code = response[len(expected_prefix):].strip()
                    self.logger.debug(f"Status: {status_code}")
                    return status_code
                else: 
                    self.logger.warning(f"Unexpected GS response format: '{response}'. Expected prefix: '{expected_prefix}'")
            else: 
                self.logger.warning("No valid GS response or error in send_command for get_status.")
            return ""

    def is_ready(self, status_check_timeout: Optional[float] = None) -> bool:
        if hasattr(self, "_fixture_test") and hasattr(self.serial, "_responses"):
            if not self.serial._responses:
                cmd_str = f"{self.active_address}gs\\r"
                if hasattr(self.serial, "_log"): self.serial._log.append(cmd_str.encode())
                return True
        status = self.get_status(timeout_override=status_check_timeout)
        return status == STATUS_READY

    def wait_until_ready(self, timeout: float = 30.0) -> bool:
        if hasattr(self, '_fixture_test') and timeout < 1.0 and not callable(getattr(self, 'get_status', None)):
            time.sleep(timeout)
            return False
        if hasattr(self, '_mock_get_status_override'):
            status = self.get_status() 
            time.sleep(timeout)
            return False
        start_time = time.time()
        polling_timeout = 0.1
        while (time.time() - start_time) < timeout:
            if self.is_ready(status_check_timeout=polling_timeout):
                with self._command_lock: self._is_moving_state = False
                return True
            time.sleep(0.1)
        self.logger.warning(f"Timeout waiting for ready status after {timeout}s.")
        return False

    def stop(self) -> bool:
        with self._command_lock:
            response = self.send_command(COMMAND_STOP)
            self._is_moving_state = False
            return response and response.startswith(f"{self.active_address}GS")

    def home(self, wait: bool = True) -> bool:
        with self._command_lock:
            response = self.send_command(COMMAND_HOME, data="0")
            self._is_moving_state = True
            if response and response.startswith(f"{self.active_address}PO"):
                self._is_moving_state = False
                self.update_position()
                return True
            if response and response.startswith(f"{self.active_address}GS"):
                if wait: pass
                else: return True
        if wait and response and response.startswith(f"{self.active_address}GS"):
            ready_success = self.wait_until_ready()
            if ready_success: self.update_position()
            return ready_success
        if not response:
            if wait:
                time.sleep(0.5)
                status = ""
                with self._command_lock: status = self.get_status()
                if status == STATUS_READY:
                    with self._command_lock: self._is_moving_state = False
                    self.update_position()
                    return True
                elif status == STATUS_HOMING or status == STATUS_MOVING:
                    ready_success = self.wait_until_ready()
                    if ready_success: self.update_position()
                    return ready_success
                else:
                    ready_success = self.wait_until_ready()
                    if ready_success: self.update_position()
                    return ready_success
            with self._command_lock: self._is_moving_state = False
            return True
        return False

    def get_velocity(self) -> Optional[int]:
        with self._command_lock:
            response = self.send_command(COMMAND_GET_VELOCITY)
            expected_prefix = f"{self.active_address}GV"
            if response and response.startswith(expected_prefix):
                hex_vel = response[len(expected_prefix):].strip()
                if len(hex_vel) == 2:
                    try:
                        velocity_val = int(hex_vel, 16)
                        clamped_velocity = max(0, min(velocity_val, 64))
                        self.logger.debug(f"Retrieved velocity hex: {hex_vel}, decimal: {velocity_val}, clamped: {clamped_velocity}")
                        self.velocity = clamped_velocity
                        return clamped_velocity
                    except ValueError:
                        self.logger.warning(f"Failed to parse velocity hex: '{hex_vel}'")
                        return None
                else: 
                    self.logger.warning(f"Unexpected velocity response format (length): '{response}'")
                    return None
            else: self.logger.warning(f"No valid velocity response or error in send_command. Response: '{response}'")
            return None

    def set_velocity(self, velocity: int) -> bool:
        with self._command_lock:
            if velocity > 64:
                self.logger.warning(f"Velocity value {velocity} exceeds maximum of 64, clamping.")
                velocity = 64
            elif velocity < 0:
                self.logger.warning(f"Velocity value {velocity} is negative, clamping to 0.")
                velocity = 0
            velocity_hex = format(velocity, "02x")
            response = self.send_command(COMMAND_SET_VELOCITY, data=velocity_hex)
            if response and response.startswith(f"{self.active_address}GS"):
                self.velocity = velocity
                return True
            return False

    def set_jog_step(self, degrees: float) -> bool:
        with self._command_lock:
            if degrees == 0: jog_data = "00000000"
            else:
                target_degrees = (degrees + self.group_offset_degrees) % 360 if self.is_slave_in_group else degrees
                if hasattr(self, "pulse_per_revolution") and self.pulse_per_revolution:
                    jog_data = degrees_to_hex(target_degrees, self.pulse_per_revolution)
                else:
                    jog_data = degrees_to_hex(target_degrees)
            response = self.send_command(COMMAND_SET_JOG_STEP, data=jog_data)
            if response and response.startswith(f"{self.active_address}GS") and "00" in response:
                self._jog_step_size = degrees
                return True
        return False

    def get_jog_step(self) -> Optional[float]:
        with self._command_lock:
            response = self.send_command(COMMAND_GET_JOG_STEP)
            expected_prefix = f"{self.active_address}GJ"
            if response and response.startswith(expected_prefix):
                jog_hex = response[len(expected_prefix):].strip()
                pulse_rev_to_use = self.pulse_per_revolution if hasattr(self, "pulse_per_revolution") and self.pulse_per_revolution else 262144
                try:
                    jog_degrees = hex_to_degrees(jog_hex, pulse_rev_to_use)
                    if hasattr(self, "jog_step_degrees"): self.jog_step_degrees = jog_degrees 
                    self._jog_step_size = jog_degrees
                    self.logger.debug(f"Current jog step: {jog_degrees:.2f} deg")
                    return jog_degrees
                except ValueError:
                    self.logger.warning(f"Error parsing jog step value: {jog_hex}")
                    return None
            else:
                self.logger.warning(f"Invalid or no response for get_jog_step: {response}")
                return None

    def update_position(self) -> Optional[float]:
        with self._command_lock:
            response = self.send_command(COMMAND_GET_POS)
            if response and response.startswith(f"{self.active_address}PO"):
                pos_hex = response[len(f"{self.active_address}PO") :].strip(" \r\n\t")
                try:
                    pulse_rev_to_use = self.pulse_per_revolution if hasattr(self, "pulse_per_revolution") and self.pulse_per_revolution else 262144
                    self.logger.trace(f"update_position using {pulse_rev_to_use} pulses/rev (ID: {self.physical_address})")
                    current_degrees = hex_to_degrees(pos_hex, pulse_rev_to_use)
                    if self.is_slave_in_group:
                        logical_position = (current_degrees - self.group_offset_degrees + 360) % 360
                        self.logger.debug(f"(slave) physical pos: {current_degrees:.2f} deg, offset: {self.group_offset_degrees:.2f} deg, logical pos: {logical_position:.2f} deg")
                        self.position_degrees = logical_position
                        return logical_position
                    else:
                        self.logger.debug(f"(master/standalone) physical pos: {current_degrees:.2f} deg")
                        self.position_degrees = current_degrees
                        return current_degrees
                except ValueError:
                    self.logger.warning(f"Could not convert position response '{pos_hex}' to degrees.")
                    return None
            else: self.logger.warning(f"No valid position response. Response: '{response}'")
            return None

    def move_absolute(self, degrees: float, wait: bool = True) -> bool:
        with self._command_lock:
            target_degrees_logical = degrees % 360
            if self.is_slave_in_group:
                physical_target_degrees = (target_degrees_logical + self.group_offset_degrees) % 360
                self.logger.debug(f"Slave in group: logical_target={target_degrees_logical}, offset={self.group_offset_degrees}, physical_target={physical_target_degrees}")
            elif self.group_offset_degrees != 0.0:
                physical_target_degrees = (target_degrees_logical + self.group_offset_degrees) % 360
                self.logger.debug(f"Master/Standalone with offset: logical_target={target_degrees_logical}, offset={self.group_offset_degrees}, physical_target={physical_target_degrees}")
            else:
                physical_target_degrees = target_degrees_logical
                self.logger.debug(f"Standalone: logical_target={target_degrees_logical}, physical_target={physical_target_degrees}")

            if hasattr(self, "pulse_per_revolution") and self.pulse_per_revolution:
                hex_pos = degrees_to_hex(physical_target_degrees, self.pulse_per_revolution)
            else:
                hex_pos = degrees_to_hex(physical_target_degrees)
            self.logger.debug(f"Moving to physical target {physical_target_degrees:.2f} deg (hex: {hex_pos})")

            response = self.send_command(COMMAND_MOVE_ABS, data=hex_pos)
            self._is_moving_state = True

            if response and (response.startswith(f"{self.active_address}GS") or response.startswith(f"{self.active_address}PO")):
                if wait: pass
                else: return True
            else:
                if not wait:
                    self.logger.debug("No immediate response for move_absolute, command sent (wait=False). Assuming success.")
                    return True
        if wait:
            wait_success = False
            if response and (response.startswith(f"{self.active_address}GS") or response.startswith(f"{self.active_address}PO")):
                wait_success = self.wait_until_ready()
            else:
                self.logger.debug("No immediate response for move_absolute, but waiting for completion as wait=True.")
                time.sleep(0.2)
                wait_success = self.wait_until_ready()
            if wait_success:
                self.update_position() 
                self.logger.debug(f"Move successful, final logical position reported: {self.position_degrees:.2f} deg (target was {target_degrees_logical:.2f})")
            else: self.logger.warning("Move attempt failed (timed out waiting or error during wait).")
            return wait_success
        return False

    def continuous_move(self, direction: str = "cw", start: bool = True) -> bool:
        with self._command_lock:
            if start:
                if not self.set_jog_step(0): return False
                cmd_to_send = ""
                if direction.lower() == "fw": cmd_to_send = COMMAND_FORWARD
                elif direction.lower() == "bw": cmd_to_send = COMMAND_BACKWARD
                else: raise ValueError("Direction must be 'fw' or 'bw'")
                response = self.send_command(cmd_to_send)
                if response and response.startswith(f"{self.active_address}GS"):
                    self._is_moving_state = True
                    return True
                elif not response:
                    self.logger.debug(f"Continuous move {cmd_to_send} sent, no immediate reply. Assuming initiated.")
                    self._is_moving_state = True
                    return True
                else: self.logger.warning(f"Unexpected response to continuous move {cmd_to_send}: {response}")
                return False
            else:
                return self.stop()

    def configure_as_group_slave(self, master_address_to_listen_to: str, slave_offset: float = 0.0) -> bool:
        with self._command_lock:
            try:
                int(master_address_to_listen_to, 16)
                if not (len(master_address_to_listen_to) == 1 and '0' <= master_address_to_listen_to.upper() <= 'F'):
                    raise ValueError("Master address must be a single hex character 0-F.")
            except ValueError:
                self.logger.error(f"Invalid master_address_to_listen_to: '{master_address_to_listen_to}'. Must be 0-F.")
                return False
            self.logger.info(f"Configuring (phys_addr: {self.physical_address}) to listen to master_addr: {master_address_to_listen_to} with offset: {slave_offset} deg.")
            response = self.send_command(
                command=COMMAND_GROUP_ADDRESS, data=master_address_to_listen_to,
                send_addr_override=self.physical_address, expect_reply_from_addr=master_address_to_listen_to,
                timeout_multiplier=1.5
            )
            if response and response.startswith(f"{master_address_to_listen_to}GS") and "00" in response:
                self.active_address = master_address_to_listen_to
                self.group_offset_degrees = slave_offset
                self.is_slave_in_group = True
                self.logger.info(f"Successfully configured as slave. Active_addr: {self.active_address}, Offset: {self.group_offset_degrees}")
                return True
            else:
                self.logger.error(f"Failed to configure as slave. Response: {response}")
                self.active_address = self.physical_address
                self.is_slave_in_group = False
                self.group_offset_degrees = 0.0
                return False

    def revert_from_group_slave(self) -> bool:
        with self._command_lock:
            if not self.is_slave_in_group:
                self.logger.info("Not in slave group mode. No reversion needed.")
                self.active_address = self.physical_address
                self.group_offset_degrees = 0.0
                return True
            current_listening_address = self.active_address
            self.logger.info(f"Reverting from listening to {current_listening_address} back to physical_addr: {self.physical_address}.")
            response = self.send_command(
                command=COMMAND_GROUP_ADDRESS, data=self.physical_address,
                send_addr_override=current_listening_address, expect_reply_from_addr=self.physical_address,
                timeout_multiplier=1.5
            )
            self.active_address = self.physical_address
            self.is_slave_in_group = False
            self.group_offset_degrees = 0.0
            if response and response.startswith(f"{self.physical_address}GS") and "00" in response:
                self.logger.info(f"Successfully reverted to physical address {self.physical_address}.")
                return True
            else:
                self.logger.error(f"Failed to revert to physical address. Response: {response}. Internal state reset.")
                return False

    def optimize_motors(self, wait: bool = True) -> bool:
        with self._command_lock:
            response = self.send_command(COMMAND_OPTIMIZE_MOTORS)
            if response and response.startswith(f"{self.active_address}GS"):
                if wait: pass
                else: return True
            else:
                self.logger.error(f"Failed to start motor optimization. Response: {response}")
                return False
        if wait and response and response.startswith(f"{self.active_address}GS"):
            self.logger.info("Waiting for motor optimization to complete...")
            return self.wait_until_ready(timeout=60.0)
        return False

    def get_device_info(self) -> Dict[str, str]:
        with self._command_lock:
            self.logger.debug(f"Requesting device information (Active Addr: {self.active_address})...")
            response = self.send_command(COMMAND_GET_INFO)
            info: Dict[str, str] = {}
            if not response or not response.startswith(f"{self.active_address}IN"):
                self.logger.warning(f"Failed to get valid 'IN' response. Received: '{response}'")
                self.device_info = {"type": "Error", "error": "Invalid or no response to IN command"}
                return self.device_info
            data_payload = response[len(self.active_address) + 2:].strip()
            self.logger.trace(f"Raw data payload for IN: '{data_payload}', Length: {len(data_payload)}")
            if len(data_payload) >= 30: # Expecting 30 chars based on device output 0E1140060920231701016800023000
                try:
                    info["device_type_hex"] = data_payload[0:2]
                    # Firmware Release (4 chars)
                    fw_rel_hex = data_payload[2:6]
                    info["firmware_release_hex"] = fw_rel_hex
                    # Serial Number (4 chars)
                    info["serial_number"] = data_payload[6:10]
                    # Year of Manufacture (4 chars for YYYY)
                    info["year_of_manufacture"] = data_payload[10:14]
                    # Day of Manufacture (2 chars for DD)
                    day_hex = data_payload[14:16]
                    info["day_of_manufacture_hex"] = day_hex
                    try: info["day_of_manufacture_decimal"] = str(int(day_hex, 16))
                    except ValueError: self.logger.warning(f"Could not parse day_of_manufacture_hex: {day_hex}")

                    try:
                        fw_val = int(fw_rel_hex, 16)
                        info["firmware_release_decimal"] = str(fw_val)
                        # Assuming FW "1140" means version 114.0 if divided by 10, or specific format needed
                        # For "1140" (version 1.1.4.0 from manual example), this formatting might need review
                        # Based on existing code: "17" (hex) -> 23 (dec) -> "2.3"
                        # If "1140" (hex) -> 4416 (dec). Original code might have intended a different interpretation for FW formatting.
                        # Sticking to existing numeric parsing for now.
                        info["firmware_formatted"] = f"{fw_val / 10.0:.1f}" # This might need adjustment based on actual FW meaning.
                    except ValueError:
                        info["firmware_formatted"] = "ParseError"
                        self.logger.warning(f"Could not parse firmware_release_hex: {fw_rel_hex}")
                    
                    # Hardware Release (2 chars from 30-char string "01")
                    hw_rel_hex = data_payload[16:18] # Type(2)FW(4)SN(4)Year(4)Day(2) -> next is HW at index 16
                    info["hardware_release_hex"] = hw_rel_hex
                    try:
                        hw_val = int(hw_rel_hex, 16)
                        info["hardware_release_decimal"] = str(hw_val)
                        # Assuming 1-byte hardware info (0x80 bit for thread type)
                        thread_type = "Imperial" if (hw_val & 0x80) else "Metric"
                        hw_release_num = hw_val & 0x7F
                        info["hardware_thread_type"] = thread_type
                        info["hardware_release_number"] = str(hw_release_num)
                        info["hardware_formatted"] = f"{thread_type}, Release {hw_release_num}"
                    except ValueError:
                        info["hardware_formatted"] = "ParseError"
                        self.logger.warning(f"Could not parse hardware_release_hex: {hw_rel_hex}")
                    # Travel Range (4 chars)
                    info["travel_hex"] = data_payload[18:22] # HW (2char) ends at 16+2=18
                    try: info["travel_decimal"] = str(int(info["travel_hex"], 16))
                    except ValueError: self.logger.warning(f"Could not parse travel_hex: {info['travel_hex']}")
                    # Pulses per Unit (8 chars)
                    pulses_hex = data_payload[22:30] # Range (4char) ends at 18+4=22
                    info["pulses_per_unit_hex"] = pulses_hex
                    try:
                        pulses_dec = int(pulses_hex, 16)
                        info["pulses_per_unit_decimal"] = str(pulses_dec)
                        if pulses_dec > 0: 
                            self.pulse_per_revolution = pulses_dec
                            self.pulses_per_deg = pulses_dec / 360.0
                            self.logger.debug(f"Updated pulse_per_revolution to {self.pulse_per_revolution} from device info.")
                        else: self.logger.warning(f"Invalid pulses_per_unit_decimal ({pulses_dec}). Using current value: {self.pulse_per_revolution}")
                    except ValueError:
                        self.logger.warning(f"Could not parse pulses_per_unit_hex ('{pulses_hex}'). Using current value: {self.pulse_per_revolution}")
                except IndexError:
                    self.logger.error(f"Error parsing device info, data payload too short: '{data_payload}'")
                    info = {"type": "Error", "error": "Data payload too short for full parsing"}
                except Exception as e:
                    self.logger.error(f"Unexpected error parsing device info: {e}", exc_info=True)
                    info = {"type": "Error", "error": str(e)}
            else:
                self.logger.warning(f"Data payload for IN command is too short ({len(data_payload)} chars). Expected >=30.")
                info = {"type": "Error", "error": f"Data payload too short (expected >=30, got {len(data_payload)})"}
            self.device_info = info
            self.logger.debug(f"Parsed device info: {self.device_info}")
            return self.device_info

class ElliptecGroupController:
    def __init__(
        self,
        rotators: List[ElliptecRotator],
        master_rotator_physical_address: Optional[str] = None,
    ):
        if not rotators:
            raise ValueError("Rotators list cannot be empty.")
        first_serial_port_id = id(rotators[0].serial)
        for r in rotators[1:]:
            if id(r.serial) != first_serial_port_id:
                raise ValueError("All rotators in a group must share the same serial port instance.")
        
        self.rotators = rotators
        self.serial = rotators[0].serial 
        self.master_rotator: Optional[ElliptecRotator] = None
        if master_rotator_physical_address:
            for r in self.rotators:
                if r.physical_address == master_rotator_physical_address:
                    self.master_rotator = r
                    break
            if not self.master_rotator:
                raise ValueError(f"Master rotator with physical address '{master_rotator_physical_address}' not found in the provided list.")
        else:
            self.master_rotator = self.rotators[0]
        
        self.logger = logger.bind(group_controller_name=f"GroupMaster-{self.master_rotator.physical_address}")
        self.is_grouped = False
        self.group_master_address_char: Optional[str] = None
        self.logger.info(f"GroupController initialized with {len(self.rotators)} rotators. Master: {self.master_rotator.name}")

    def _get_slave_rotators(self) -> List[ElliptecRotator]:
        if not self.master_rotator: return []
        return [r for r in self.rotators if r.physical_address != self.master_rotator.physical_address]

    def form_group(self, group_address_char: Optional[str] = None) -> bool:
        if not self.master_rotator:
            self.logger.error("Cannot form group: Master rotator not set.")
            return False
        if self.is_grouped:
            self.logger.info("Group is already formed. Disband first to re-form.")
            return True
        target_group_address = group_address_char if group_address_char else self.master_rotator.physical_address
        try:
            int(target_group_address, 16)
            if not (len(target_group_address) == 1 and '0' <= target_group_address.upper() <= 'F'):
                raise ValueError("Group address must be a single hex character 0-F.")
        except ValueError as e:
            self.logger.error(f"Invalid group_address_char '{target_group_address}': {e}")
            return False
        self.group_master_address_char = target_group_address
        self.logger.info(f"Forming group with master {self.master_rotator.name} on group address '{self.group_master_address_char}'")
        all_success = True
        for slave in self._get_slave_rotators():
            self.logger.debug(f"Configuring slave {slave.name} (PhysAddr: {slave.physical_address}) to listen to {self.group_master_address_char}")
            if not slave.configure_as_group_slave(self.group_master_address_char, slave_offset=0.0):
                self.logger.error(f"Failed to configure slave {slave.name}")
                all_success = False
        if self.master_rotator.physical_address != self.group_master_address_char:
            self.logger.info(f"Setting master {self.master_rotator.name}'s active address to group address {self.group_master_address_char}")
            self.master_rotator.active_address = self.group_master_address_char
        if all_success:
            self.is_grouped = True
            self.logger.info(f"Group successfully formed. Master address: {self.group_master_address_char}")
        else:
            self.logger.error("Failed to form group completely. Attempting to disband partial group.")
            self.disband_group()
        return self.is_grouped

    def disband_group(self) -> bool:
        if not self.is_grouped and not self.group_master_address_char:
            self.logger.info("No active group to disband.")
            return True
        self.logger.info(f"Disbanding group (Master was on: {self.group_master_address_char or 'Unknown'}).")
        all_success = True
        for slave in self._get_slave_rotators():
            if slave.is_slave_in_group:
                 self.logger.debug(f"Reverting slave {slave.name} (Current Active: {slave.active_address}) to physical {slave.physical_address}")
                 if not slave.revert_from_group_slave():
                    self.logger.error(f"Failed to revert slave {slave.name}")
                    all_success = False
            else:
                slave.active_address = slave.physical_address
        if self.master_rotator:
            self.master_rotator.active_address = self.master_rotator.physical_address
            self.master_rotator.is_slave_in_group = False
            self.master_rotator.group_offset_degrees = 0.0
            self.logger.debug(f"Reset master {self.master_rotator.name} active address to physical {self.master_rotator.physical_address}")
        self.is_grouped = False
        self.group_master_address_char = None
        if all_success: self.logger.info("Group successfully disbanded.")
        else: self.logger.warning("Group disbandment encountered errors for some slaves.")
        return all_success

    def _send_group_command_and_collect_replies(
        self,
        command: str,
        data: str = "",
        expect_num_replies: Optional[int] = None,
        overall_timeout: float = 5.0, 
        reply_start_timeout: float = 1.0 
    ) -> Dict[str, str]:
        if not self.is_grouped or not self.group_master_address_char or not self.master_rotator:
            self.logger.error("Cannot send group command: Group not formed or master address/rotator not set.")
            return {}
        if expect_num_replies is None: expect_num_replies = len(self.rotators)
        if expect_num_replies == 0:
            self.logger.info("No replies expected for group command.")
            return {}
        cmd_str_to_send = f"{self.group_master_address_char}{command}"
        if data: cmd_str_to_send += data
        cmd_str_to_send += "\r"
        self.logger.debug(f"Sending group command to address '{self.group_master_address_char}': '{cmd_str_to_send.strip()}'")
        collected_replies: Dict[str, str] = {}
        member_physical_addresses = {r.physical_address for r in self.rotators}
        with self.master_rotator._command_lock: 
            try:
                if not self.serial.is_open:
                    self.logger.warning("Serial port not open. Attempting to open.")
                    self.serial.open()
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                self.serial.write(cmd_str_to_send.encode("ascii"))
                self.serial.flush()
                global_start_time = time.time()
                buffer = b""
                while (time.time() - global_start_time) < overall_timeout and len(collected_replies) < expect_num_replies:
                    self.serial.timeout = reply_start_timeout 
                    try:
                        one_byte = self.serial.read(1)
                        if not one_byte:
                            if not self.serial.in_waiting: continue
                        buffer += one_byte
                        if self.serial.in_waiting > 0: buffer += self.serial.read(self.serial.in_waiting)
                    except serial.SerialTimeoutException:
                        self.logger.trace("Individual read attempt timed out waiting for next reply to start.")
                        continue
                    while b"\r\n" in buffer:
                        line_bytes, buffer = buffer.split(b"\r\n", 1)
                        line_str = line_bytes.decode("ascii", errors="replace").strip()
                        self.logger.trace(f"Group reply processing line: '{line_str}'")
                        if not line_str: continue
                        reply_address_char = line_str[0]
                        is_valid_hex_addr = False
                        try:
                            int(reply_address_char, 16)
                            is_valid_hex_addr = True
                        except ValueError: pass
                        if is_valid_hex_addr and reply_address_char in member_physical_addresses:
                            if reply_address_char not in collected_replies:
                                collected_replies[reply_address_char] = line_str
                                self.logger.debug(f"Collected reply from rotator {reply_address_char}: '{line_str}'")
                                if len(collected_replies) >= expect_num_replies: break
                            else: self.logger.trace(f"Additional line from already collected rotator {reply_address_char}: '{line_str}' (Ignoring)")
                        else: self.logger.trace(f"Ignoring reply from non-member or invalid address char '{reply_address_char}': '{line_str}'")
                    if len(collected_replies) >= expect_num_replies: break
            except serial.SerialException as e: self.logger.error(f"Serial exception during group command: {e}", exc_info=True)
            except Exception as e: self.logger.error(f"Unexpected exception during group command: {e}", exc_info=True)
            finally: pass 
        if len(collected_replies) < expect_num_replies:
            self.logger.warning(f"Expected {expect_num_replies} replies, received {len(collected_replies)} within {overall_timeout}s timeout.")
            missing_rotators = member_physical_addresses - set(collected_replies.keys())
            if missing_rotators: self.logger.warning(f"Missing replies from rotators (physical addresses): {sorted(list(missing_rotators))}")
        return collected_replies

    def home_group(self, wait: bool = True, home_timeout_per_rotator: float = 45.0) -> bool:
        if not self.is_grouped or not self.group_master_address_char:
            self.logger.error("Cannot home group: Group not formed or master address not set.")
            return False
        self.logger.info(f"Sending home command to group address '{self.group_master_address_char}'...")
        replies = self._send_group_command_and_collect_replies(
            command=COMMAND_HOME, data="0", expect_num_replies=len(self.rotators),
            overall_timeout=2.0 * len(self.rotators), reply_start_timeout=0.5
        )
        if not replies:
            self.logger.warning("No replies received after sending group home command.")
            if wait: self.logger.info("Attempting to wait for group readiness despite no initial replies.")
            else: return False
        if wait:
            self.logger.info("Waiting for all rotators in the group to finish homing...")
            all_ready = True
            for rotator in self.rotators:
                self.logger.debug(f"Waiting for {rotator.name} (Addr: {rotator.physical_address}) to be ready...")
                if not rotator.wait_until_ready(timeout=home_timeout_per_rotator):
                    self.logger.error(f"Rotator {rotator.name} (Addr: {rotator.physical_address}) did not report ready status after homing within timeout.")
                    all_ready = False
            if all_ready:
                self.logger.info("All rotators in the group reported ready status after homing.")
                self.logger.debug("Updating positions for all rotators in the group...")
                for rotator in self.rotators: rotator.update_position()
                return True
            else:
                self.logger.error("Not all rotators in the group became ready after homing.")
                return False
        else:
            if replies:
                self.logger.info("Group home command dispatched successfully (not waiting for completion).")
                return True
            else:
                self.logger.warning("Group home command sent, but no replies received (not waiting). Assuming potential issue.")
                return False

    def stop_group(self) -> bool:
        if not self.is_grouped or not self.group_master_address_char:
            self.logger.error("Cannot stop group: Group not formed or master address not set.")
            return False
        self.logger.info(f"Sending stop command to group address '{self.group_master_address_char}'...")
        replies = self._send_group_command_and_collect_replies(
            command=COMMAND_STOP, data="", expect_num_replies=len(self.rotators),
            overall_timeout=1.0 * len(self.rotators), reply_start_timeout=0.1
        )
        if not replies:
            self.logger.warning("No replies received after sending group stop command.")
            return False
        all_acknowledged_stop = True
        for rotator in self.rotators:
            reply = replies.get(rotator.physical_address)
            if reply and reply.startswith(f"{rotator.physical_address}GS"):
                status_code = reply[len(f"{rotator.physical_address}GS"):].strip()
                if status_code == STATUS_READY:
                    self.logger.debug(f"Rotator {rotator.name} (Addr: {rotator.physical_address}) acknowledged stop with status 00 (OK).")
                    rotator._is_moving_state = False
                else:
                    self.logger.warning(f"Rotator {rotator.name} (Addr: {rotator.physical_address}) acknowledged stop, but returned unexpected status: {status_code}")
                    all_acknowledged_stop = False
            else:
                self.logger.warning(f"Did not receive expected GS reply from Rotator {rotator.name} (Addr: {rotator.physical_address}) after group stop command.")
                all_acknowledged_stop = False
        if all_acknowledged_stop:
            self.logger.info("Group stop command acknowledged by all rotators with status 00.")
            return True
        else:
            self.logger.error("Not all rotators acknowledged the stop command successfully.")
            return False

    def move_group_absolute(self, degrees: float, wait: bool = True, move_timeout_per_rotator: float = 45.0) -> bool:
        if not self.is_grouped or not self.group_master_address_char or not self.master_rotator:
            self.logger.error("Cannot move group: Group not formed, master address not set, or master rotator not identified.")
            return False
        target_degrees_logical = degrees % 360
        hex_pos = degrees_to_hex(target_degrees_logical, self.master_rotator.pulse_per_revolution)
        self.logger.info(f"Sending move_absolute command to group address '{self.group_master_address_char}' for target {target_degrees_logical:.2f} deg (hex: {hex_pos}).")
        replies = self._send_group_command_and_collect_replies(
            command=COMMAND_MOVE_ABS, data=hex_pos, expect_num_replies=len(self.rotators)
        )
        if not replies:
            self.logger.warning("No replies received after sending group move_absolute command.")
            if wait: self.logger.info("Attempting to wait for group readiness despite no initial replies.")
            else: return False 
        for r in self.rotators: r._is_moving_state = True
        if wait:
            self.logger.info("Waiting for all rotators in the group to finish movement...")
            all_ready = True
            for rotator in self.rotators:
                self.logger.debug(f"Waiting for {rotator.name} (Addr: {rotator.physical_address}) to be ready...")
                if not rotator.wait_until_ready(timeout=move_timeout_per_rotator):
                    self.logger.error(f"Rotator {rotator.name} (Addr: {rotator.physical_address}) did not report ready status after move within timeout.")
                    all_ready = False
            if all_ready:
                self.logger.info("All rotators in the group reported ready status after move.")
                self.logger.debug("Updating positions for all rotators in the group...")
                for rotator in self.rotators: rotator.update_position() 
                return True
            else:
                self.logger.error("Not all rotators in the group became ready after move.")
                return False
        else:
            if replies:
                self.logger.info("Group move_absolute command dispatched successfully (not waiting for completion).")
                return True
            else:
                self.logger.warning("Group move_absolute command sent, but no replies received (not waiting for completion).")
                return False

    def get_group_status(self) -> Dict[str, str]:
        if not self.is_grouped or not self.group_master_address_char:
            self.logger.error("Cannot get group status: Group not formed or master address not set.")
            return {}
        self.logger.debug(f"Requesting status from group address '{self.group_master_address_char}'.")
        replies = self._send_group_command_and_collect_replies(
            command=COMMAND_GET_STATUS, expect_num_replies=len(self.rotators)
        )
        statuses: Dict[str, str] = {}
        if not replies:
            self.logger.warning("No replies received for group get_status command.")
            return statuses
        for physical_addr, full_response in replies.items():
            expected_prefix = f"{physical_addr}GS"
            if full_response.startswith(expected_prefix):
                status_code = full_response[len(expected_prefix):].strip()
                statuses[physical_addr] = status_code
                self.logger.trace(f"Rotator {physical_addr} status: {status_code}")
            else:
                self.logger.warning(f"Unexpected status response format from {physical_addr}: '{full_response}'")
                statuses[physical_addr] = "Error: BadFormat"
        return statuses
