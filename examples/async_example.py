#!/usr/bin/env python3
"""
Asynchronous Usage Example for Elliptec Controller

This example demonstrates how to use the asynchronous features of the 
ElliptecRotator class for non-blocking device communication.

The implementation uses per-command response queues for reliable command handling,
improving error isolation and preventing command/response mismatches.
"""

import time
import sys
from loguru import logger
from elliptec_controller import ElliptecRotator

# Configure logging
logger.remove()
logger.add(sys.stderr, level="INFO")

def async_example_context_manager():
    """Example using context manager for automatic thread management"""
    logger.info("=== Asynchronous Example with Context Manager ===")
    
    # Using context manager to automatically handle connect/disconnect
    with ElliptecRotator(
        port="/dev/ttyUSB0",  # Replace with your port
        motor_address=1,      # Replace with your device address
        name="AsyncRotator"
    ) as rotator:
        # Get initial position
        current_pos = rotator.update_position()
        logger.info(f"Initial position: {current_pos:.2f}°")
        
        # Move to 0 degrees as a reference point
        logger.info("Moving to 0 degrees (blocking)...")
        rotator.move_absolute(0.0, wait=True)
        
        # Start a non-blocking move operation
        target_angle = 45.0
        logger.info(f"Starting move to {target_angle}° (non-blocking)...")
        start_time = time.time()
        
        # Move without waiting for completion
        rotator.move_absolute(target_angle, wait=False)
        
        # Simulate doing other work while movement happens
        logger.info("Doing other work while device is moving...")
        
        # Monitor movement status periodically
        dots = 0
        while rotator.is_moving:
            sys.stdout.write(".")
            sys.stdout.flush()
            dots += 1
            if dots % 10 == 0:
                # Check position occasionally during movement
                pos = rotator.update_position()
                logger.info(f"Current position during movement: {pos:.2f}°")
            time.sleep(0.1)
        
        # Movement complete
        elapsed = time.time() - start_time
        final_pos = rotator.update_position()
        logger.info(f"\nMove complete! Position: {final_pos:.2f}°, took {elapsed:.2f} seconds")
        
        # Demonstrate mixing sync and async modes
        logger.info("Demonstrating mixed sync/async modes...")
        
        # Force synchronous mode for a specific operation
        logger.info("Moving to 90° (explicitly synchronous)...")
        rotator.move_absolute(90.0, use_async=False)
        logger.info("Synchronous move complete")
        
        # Force asynchronous mode for a specific operation
        logger.info("Moving to 180° (explicitly asynchronous)...")
        rotator.move_absolute(180.0, use_async=True)
        
        # Each command uses its own private response queue behind the scenes
        # This prevents response mixups when multiple commands are issued rapidly
        
        # Wait until ready using the dedicated method
        logger.info("Waiting for async move to complete...")
        rotator.wait_until_ready()
        logger.info(f"Final position: {rotator.position_degrees:.2f}°")
        
        # Thread will automatically be stopped when exiting the context manager
    
    logger.info("Context manager exited, thread automatically stopped")

def async_example_manual():
    """Example using manual thread management"""
    logger.info("\n=== Asynchronous Example with Manual Thread Management ===")
    
    # Create rotator instance
    rotator = ElliptecRotator(
        port="/dev/ttyUSB0",  # Replace with your port
        motor_address=1,      # Replace with your device address
        name="ManualAsyncRotator"
    )
    
    try:
        # Manually start the async thread
        logger.info("Manually starting async thread...")
        rotator.connect()
        
        # Perform operations (async by default after connect)
        current_pos = rotator.update_position()
        logger.info(f"Current position: {current_pos:.2f}°")
        
        # Start move operation
        target_angle = 45.0
        logger.info(f"Moving to {target_angle}° (async)...")
        # After connect(), async mode is default (_use_async=True is set in connect())
        rotator.move_absolute(target_angle)
        
        # Monitor status with a timeout
        timeout = 5.0
        start_time = time.time()
        while rotator.is_moving and (time.time() - start_time) < timeout:
            sys.stdout.write(".")
            sys.stdout.flush()
            time.sleep(0.1)
            
        print()  # New line after dots
        
        # Check if movement completed or timed out
        if rotator.is_moving:
            logger.warning(f"Movement timed out after {timeout} seconds!")
        else:
            logger.info(f"Movement complete, position: {rotator.position_degrees:.2f}°")
            
    finally:
        # Always disconnect to clean up the thread
        logger.info("Manually stopping async thread...")
        rotator.disconnect()
        logger.info("Thread stopped")

def multiple_rotators_example():
    """Example of controlling multiple rotators asynchronously"""
    logger.info("\n=== Multiple Rotators Asynchronous Example ===")
    
    # Set up multiple rotators on the same port
    # Assuming you have devices at addresses 1 and 2
    rotator1 = ElliptecRotator(
        port="/dev/ttyUSB0",  # Replace with your port
        motor_address=1,
        name="Rotator1"
    )
    
    rotator2 = ElliptecRotator(
        port="/dev/ttyUSB0",  # Same port
        motor_address=2,
        name="Rotator2"
    )
    
    try:
        # Start async threads for both
        logger.info("Starting async threads for both rotators...")
        rotator1.connect()
        rotator2.connect()
        
        # Move both devices simultaneously
        logger.info("Moving Rotator1 to 45°...")
        rotator1.move_absolute(45.0, wait=False)
        
        logger.info("Moving Rotator2 to 90° (both moves happening in parallel)...")
        rotator2.move_absolute(90.0, wait=False)
        
        # Wait for both to complete
        logger.info("Waiting for both movements to complete...")
        # The worker threads handle each command independently with separate response queues
        # This allows truly parallel operation even on a shared serial port
        waiting = True
        while waiting:
            r1_ready = not rotator1.is_moving
            r2_ready = not rotator2.is_moving
            
            if r1_ready and r2_ready:
                waiting = False
            
            status = f"Rotator1: {'READY' if r1_ready else 'MOVING'}, " \
                     f"Rotator2: {'READY' if r2_ready else 'MOVING'}"
            logger.info(status)
            
            # Only wait if still waiting
            if waiting:
                time.sleep(0.5)
        
        # Get final positions
        r1_pos = rotator1.update_position()
        r2_pos = rotator2.update_position()
        logger.info(f"Final positions: Rotator1 = {r1_pos:.2f}°, Rotator2 = {r2_pos:.2f}°")
        
    finally:
        # Always disconnect both
        logger.info("Stopping async threads...")
        rotator1.disconnect()
        rotator2.disconnect()
        logger.info("Threads stopped")

if __name__ == "__main__":
    logger.info("Elliptec Controller Asynchronous Examples")
    logger.info("Implementation features:")
    logger.info(" - Per-command response queues for reliability")
    logger.info(" - Context manager for automatic thread lifecycle")
    logger.info(" - Compatible with both sync and async operation modes")
    logger.info(" - Improved error isolation and handling")
    
    try:
        # Run the examples
        async_example_context_manager()
        async_example_manual()
        
        # Uncomment to run the multiple rotators example if you have multiple devices
        # multiple_rotators_example()
        
        logger.info("\nAll examples completed successfully!")
        
    except Exception as e:
        logger.error(f"Error running examples: {e}", exc_info=True)
        sys.exit(1)