#!/usr/bin/env python3
"""
Basic demonstration of the elliptec-controller package.

This example shows how to initialize and control a single rotator
and how to work with a triple rotator setup.
"""

import time
import serial
from elliptec_controller import ElliptecRotator, TripleRotatorController

def single_rotator_demo(port_name="/dev/ttyUSB0"):
    """
    Demonstrate control of a single rotator.
    
    Args:
        port_name: Serial port where the rotator is connected
    """
    print("=== Single Rotator Demo ===")
    
    # Open a serial connection
    ser = serial.Serial(port_name, baudrate=9600, timeout=1)
    
    # Create a rotator instance
    rotator = ElliptecRotator(ser, motor_address=1, name="TestRotator")
    
    try:
        # Get device info
        device_info = rotator.get_device_info(debug=True)
        print(f"Device info: {device_info}")
        
        # Home the rotator
        print("Homing rotator...")
        rotator.home(wait=True)
        print("Homing complete")
        
        # Move to specific positions
        for angle in [0, 45, 90, 135, 180]:
            print(f"Moving to {angle} degrees...")
            rotator.move_absolute(angle, wait=True)
            print(f"Position reached: {angle} degrees")
            time.sleep(1)
        
        # Return to home
        print("Returning to home position...")
        rotator.home(wait=True)
        print("Homing complete")
        
    finally:
        # Close the serial port
        ser.close()
        print("Serial connection closed")

def triple_rotator_demo(port_name="/dev/ttyUSB0"):
    """
    Demonstrate control of a triple rotator setup.
    
    Args:
        port_name: Serial port where the rotators are connected
    """
    print("\n=== Triple Rotator Demo ===")
    
    # Create a controller with three rotators
    controller = TripleRotatorController(
        port=port_name,
        addresses=[3, 6, 8],
        names=["HWP1", "QWP", "HWP2"]
    )
    
    try:
        # Check if all rotators are ready
        ready = controller.is_all_ready()
        print(f"All rotators ready: {ready}")
        
        # Home all rotators
        print("Homing all rotators...")
        controller.home_all(wait=True)
        print("Homing complete")
        
        # Set velocities
        print("Setting velocities...")
        controller.set_all_velocities(40)
        print("Velocities set")
        
        # Move to specific positions
        print("Moving to positions...")
        controller.move_all_absolute([30, 45, 60], wait=True)
        print("Positions reached")
        
        # Wait a moment
        time.sleep(2)
        
        # Move relative
        print("Moving relative...")
        controller.move_all_relative([10, 15, 20], ["cw", "cw", "ccw"], wait=True)
        print("Relative movement complete")
        
        # Return to home
        print("Returning to home positions...")
        controller.home_all(wait=True)
        print("Homing complete")
        
    finally:
        # Close the controller
        controller.close()
        print("Controller closed")

if __name__ == "__main__":
    # Replace with your actual serial port
    PORT = "/dev/ttyUSB0"
    
    # Run demos - uncomment as needed
    # single_rotator_demo(PORT)
    # triple_rotator_demo(PORT)