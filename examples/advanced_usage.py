#!/usr/bin/env python3
"""
Advanced demonstration of the elliptec-controller package.

This example shows more complex operations including error handling,
continuous motion, and device information retrieval.
"""

import time
import serial
import signal
import sys
from contextlib import contextmanager
from elliptec_controller import TripleRotatorController


class TimeoutError(Exception):
    """Raised when an operation times out."""
    pass


@contextmanager
def time_limit(seconds):
    """
    Context manager to limit execution time of a block of code.
    
    Args:
        seconds: Maximum allowed execution time in seconds
    """
    def signal_handler(signum, frame):
        raise TimeoutError("Operation timed out")
    
    signal.signal(signal.SIGALRM, signal_handler)
    signal.alarm(seconds)
    try:
        yield
    finally:
        signal.alarm(0)  # Disable the alarm


def sweep_polarization(controller, start_angles, end_angles, steps=10):
    """
    Sweep through a range of polarization states.
    
    Args:
        controller: TripleRotatorController instance
        start_angles: List of starting angles for each rotator
        end_angles: List of ending angles for each rotator
        steps: Number of steps for the sweep
    """
    if (len(start_angles) != len(controller.rotators) or 
            len(end_angles) != len(controller.rotators)):
        raise ValueError("Need start and end angles for each rotator")
    
    try:
        # Check all rotators are ready
        if not controller.is_all_ready():
            print("Not all rotators are ready. Homing first...")
            controller.home_all(wait=True)
        
        # Calculate angle steps
        angle_steps = []
        for i in range(len(start_angles)):
            angle_steps.append((end_angles[i] - start_angles[i]) / steps)
        
        # Move to start position
        print(f"Moving to start position: {start_angles}")
        controller.move_all_absolute(start_angles, wait=True)
        
        # Perform the sweep
        for step in range(steps + 1):
            current_angles = [
                start_angles[i] + angle_steps[i] * step 
                for i in range(len(start_angles))
            ]
            print(f"Step {step}/{steps}: Moving to {current_angles}")
            controller.move_all_absolute(current_angles, wait=True)
            
            # Perform your measurement here
            print("Measuring...")
            time.sleep(0.5)  # Simulate a measurement
        
        print("Sweep complete!")
    
    except Exception as e:
        print(f"Error during sweep: {e}")
        return False
    
    return True


def advanced_demo(port_name="/dev/ttyUSB0"):
    """
    Run an advanced demonstration of the controller capabilities.
    
    Args:
        port_name: Serial port name
    """
    print("=== Advanced Elliptec Controller Demo ===")
    
    # Create controller with error handling
    controller = None
    try:
        controller = TripleRotatorController(
            port=port_name,
            addresses=[3, 6, 8],
            names=["HWP1", "QWP", "HWP2"]
        )
    except Exception as e:
        print(f"Failed to initialize controller: {e}")
        return
    
    try:
        # Get device information
        print("\n=== Device Information ===")
        for i, rotator in enumerate(controller.rotators):
            print(f"Rotator {i+1} ({rotator.name}):")
            try:
                with time_limit(3):  # Set timeout for device info request
                    info = rotator.get_device_info(debug=True)
                    for key, value in info.items():
                        print(f"  {key}: {value}")
            except TimeoutError:
                print("  Timed out while getting device info")
            except Exception as e:
                print(f"  Error getting device info: {e}")
        
        # Home all rotators with timeout protection
        print("\n=== Homing Rotators ===")
        try:
            with time_limit(30):  # 30-second timeout for homing
                result = controller.home_all(wait=True)
                print(f"Homing result: {result}")
        except TimeoutError:
            print("Homing timed out. Stopping all rotators.")
            controller.stop_all()
        
        # Perform a polarization sweep
        print("\n=== Polarization Sweep ===")
        start_angles = [0, 0, 0]      # All at 0 degrees
        end_angles = [90, 45, 90]     # Final positions
        sweep_result = sweep_polarization(
            controller, 
            start_angles, 
            end_angles, 
            steps=5
        )
        print(f"Sweep completed successfully: {sweep_result}")
        
        # Demonstrate velocity control
        print("\n=== Velocity Control ===")
        print("Setting low velocity")
        controller.set_all_velocities(20)
        
        print("Moving with low velocity")
        controller.move_all_absolute([45, 45, 45], wait=True)
        
        print("Setting high velocity")
        controller.set_all_velocities(50)
        
        print("Moving with high velocity")
        controller.move_all_absolute([0, 0, 0], wait=True)
        
        # Clean up - return to home position
        print("\n=== Cleanup ===")
        controller.home_all(wait=True)
    
    except Exception as e:
        print(f"Demonstration error: {e}")
    
    finally:
        # Always close the controller
        if controller:
            controller.close()
            print("Controller closed")


if __name__ == "__main__":
    # Replace with your actual serial port
    PORT = "/dev/ttyUSB0"
    
    # Run the advanced demo
    advanced_demo(PORT)