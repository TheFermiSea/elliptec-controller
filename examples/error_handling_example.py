#!/usr/bin/env python3
"""
Error Handling Example for Elliptec Controller

This example demonstrates robust error handling techniques when using the Elliptec Controller,
especially in asynchronous mode. It shows how to properly handle various error conditions
that might occur during device communication.
"""

import time
import sys
import serial
from loguru import logger
from elliptec_controller import ElliptecRotator, ElliptecError

# Configure logging
logger.remove()
logger.add(sys.stderr, level="INFO")

def demonstrate_basic_error_handling():
    """Show basic error handling techniques"""
    logger.info("=== Basic Error Handling Example ===")
    
    # Example of handling connection errors
    try:
        # Intentionally use an invalid port name
        rotator = ElliptecRotator(
            port="/dev/nonexistent_port",
            motor_address=1,
            name="ErrorRotator"
        )
    except serial.SerialException as e:
        logger.error(f"Serial connection error: {e}")
        logger.info("✓ Successfully caught connection error")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    
    # Example of handling device operation errors
    try:
        # Use a valid port but handle potential operation errors
        # Replace with your actual port
        port_name = "/dev/ttyUSB0"
        
        logger.info(f"Attempting to connect to {port_name}")
        rotator = ElliptecRotator(
            port=port_name,
            motor_address=1,
            name="ErrorRotator",
            # Disable auto-home to prevent errors during initialization
            auto_home=False
        )
        
        # Execute operations with error checking
        logger.info("Checking status...")
        status = rotator.get_status()
        if not status:
            logger.warning("Failed to get status, device might be disconnected")
            return
            
        logger.info(f"Device status: {status}")
        
        # Try moving with proper error handling
        logger.info("Moving to 45 degrees...")
        if not rotator.move_absolute(45.0, wait=True):
            logger.error("Movement failed!")
            # Recovery action: attempt to home
            logger.info("Attempting recovery by homing...")
            if rotator.home(wait=True):
                logger.info("Recovery successful")
            else:
                logger.error("Recovery failed, device might need attention")
        else:
            logger.info("Movement successful")
            
    except serial.SerialException as e:
        logger.error(f"Serial error during operation: {e}")
    except Exception as e:
        logger.error(f"Unexpected error during operation: {e}")

def demonstrate_async_error_handling():
    """Demonstrate error handling in asynchronous mode"""
    logger.info("\n=== Asynchronous Error Handling Example ===")
    
    # Replace with your actual port
    port_name = "/dev/ttyUSB0"
    
    # Create rotator instance
    try:
        rotator = ElliptecRotator(
            port=port_name,
            motor_address=1,
            name="AsyncErrorRotator",
            auto_home=False
        )
    except Exception as e:
        logger.error(f"Failed to create rotator: {e}")
        return
    
    try:
        # Start async thread
        logger.info("Starting async thread...")
        rotator.connect()
        
        # Example 1: Handling command failures in async mode
        try:
            logger.info("Sending an invalid command...")
            # This will raise an ElliptecError if the device is properly connected
            # but handles it gracefully if not
            response = rotator.send_command("zz", use_async=True)  # Invalid command
            logger.info(f"Response to invalid command: '{response}'")
            if not response:
                logger.warning("Command failed as expected")
        except ElliptecError as e:
            logger.info(f"✓ Successfully caught command error: {e}")
        
        # Example 2: Handling timeout in async mode
        logger.info("Testing timeout handling...")
        try:
            # Attempt a move with a very short timeout (likely to timeout)
            response = rotator._send_command_async(
                command="ma",
                data="12345678",  # Some position data
                timeout=0.001  # Unrealistically short timeout
            )
            if not response:
                logger.info("✓ Timeout handled gracefully")
        except Exception as e:
            logger.error(f"Timeout handling failed: {e}")
        
        # Example 3: Recovering from errors during movement
        logger.info("Testing movement error recovery...")
        success = rotator.move_absolute(180.0, wait=False)
        
        # Simulate detecting a problem during movement
        time.sleep(0.1)
        logger.warning("Simulated problem detected during movement!")
        
        # Emergency stop
        logger.info("Executing emergency stop...")
        rotator.stop()
        
        # Wait for device to settle
        time.sleep(0.5)
        
        # Check status
        if not rotator.is_moving:
            logger.info("✓ Movement successfully stopped")
        else:
            logger.error("Failed to stop movement")
        
        # Recovery: home the device
        logger.info("Attempting recovery by homing...")
        if rotator.home(wait=True):
            logger.info("✓ Recovery successful")
        else:
            logger.error("Recovery failed")
        
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        # Always disconnect to clean up resources
        logger.info("Disconnecting async thread...")
        rotator.disconnect()
        logger.info("Async thread stopped")

def demonstrate_multiple_device_error_handling():
    """Demonstrate error handling with multiple devices"""
    logger.info("\n=== Multiple Device Error Handling Example ===")
    
    # Replace with your actual port
    port_name = "/dev/ttyUSB0"
    
    rotator1 = None
    rotator2 = None
    
    try:
        # Create first rotator
        logger.info("Creating first rotator...")
        rotator1 = ElliptecRotator(
            port=port_name,
            motor_address=1,
            name="Rotator1",
            auto_home=False
        )
        
        # Create second rotator
        logger.info("Creating second rotator...")
        rotator2 = ElliptecRotator(
            port=port_name,
            motor_address=2,  # If you don't have a second device, this could cause errors
            name="Rotator2",
            auto_home=False
        )
        
        # Start async threads for both
        logger.info("Starting async threads...")
        rotator1.connect()
        rotator2.connect()
        
        # Try to move both simultaneously
        logger.info("Moving both rotators simultaneously...")
        
        # Start both movements
        r1_success = rotator1.move_absolute(45.0, wait=False)
        r2_success = rotator2.move_absolute(90.0, wait=False)
        
        if not r1_success or not r2_success:
            logger.warning("One or both movements failed to start")
            if not r1_success:
                logger.error("Rotator1 movement failed")
            if not r2_success:
                logger.error("Rotator2 movement failed")
        
        # Monitor both devices
        timeout = 5.0
        start_time = time.time()
        
        try:
            while (time.time() - start_time) < timeout:
                r1_status = "READY" if not rotator1.is_moving else "MOVING"
                
                # Intentionally access rotator2 in a try block in case it fails
                try:
                    r2_status = "READY" if not rotator2.is_moving else "MOVING"
                except Exception:
                    r2_status = "ERROR"
                
                logger.info(f"Status - Rotator1: {r1_status}, Rotator2: {r2_status}")
                
                if r1_status == "READY" and r2_status in ["READY", "ERROR"]:
                    break
                    
                time.sleep(0.5)
                
            if (time.time() - start_time) >= timeout:
                logger.warning("Operation timed out")
                
                # Emergency stop both rotators
                logger.info("Executing emergency stop on both rotators...")
                try:
                    rotator1.stop()
                    logger.info("Rotator1 stopped")
                except Exception as e:
                    logger.error(f"Failed to stop Rotator1: {e}")
                
                try:
                    rotator2.stop()
                    logger.info("Rotator2 stopped")
                except Exception as e:
                    logger.error(f"Failed to stop Rotator2: {e}")
            else:
                logger.info("Operation completed within timeout")
                
        except Exception as e:
            logger.error(f"Error during monitoring: {e}")
            
    except Exception as e:
        logger.error(f"Setup error: {e}")
    finally:
        # Ensure both rotators are properly disconnected
        logger.info("Cleaning up...")
        
        if rotator1 is not None:
            try:
                rotator1.disconnect()
                logger.info("Rotator1 disconnected")
            except Exception as e:
                logger.error(f"Error disconnecting Rotator1: {e}")
        
        if rotator2 is not None:
            try:
                rotator2.disconnect()
                logger.info("Rotator2 disconnected")
            except Exception as e:
                logger.error(f"Error disconnecting Rotator2: {e}")

def demonstrate_context_manager_error_handling():
    """Demonstrate error handling with context manager"""
    logger.info("\n=== Context Manager Error Handling Example ===")
    
    # Replace with your actual port
    port_name = "/dev/ttyUSB0"
    
    try:
        logger.info("Using context manager for automatic resource management...")
        
        # The context manager ensures disconnect() is called even if exceptions occur
        with ElliptecRotator(
            port=port_name,
            motor_address=1,
            name="ContextRotator",
            auto_home=False
        ) as rotator:
            logger.info("Context manager initialized rotator and started async thread")
            
            # Normal operations
            status = rotator.get_status()
            logger.info(f"Device status: {status}")
            
            # Simulate an error condition
            logger.info("Simulating an error condition...")
            try:
                # Raise an arbitrary exception
                raise RuntimeError("Simulated error during operation")
            except Exception as e:
                logger.error(f"Caught error: {e}")
                logger.info("✓ Operations can continue despite error")
            
            # Continue operations after error
            logger.info("Continuing operations after error...")
            rotator.move_absolute(30.0, wait=False)
            time.sleep(0.5)
            
            # Simulate another error
            logger.info("Simulating another error...")
            try:
                # Invalid position value
                rotator.move_absolute(-999999.0, wait=True)
            except Exception as e:
                logger.error(f"Caught movement error: {e}")
                logger.info("✓ Error properly contained")
            
            logger.info("Completing context block - thread will automatically stop")
            
        logger.info("Context manager successfully cleaned up resources despite errors")
        
    except Exception as e:
        logger.error(f"Unexpected error in context manager example: {e}")
        logger.info("Even with this error, resource cleanup would still occur")

if __name__ == "__main__":
    logger.info("Elliptec Controller Error Handling Examples")
    logger.info("These examples demonstrate robust error handling techniques")
    logger.info("Note: Some examples intentionally cause errors to show handling")
    logger.info("----------------------------------------")
    
    try:
        # Demonstration of different error handling scenarios
        demonstrate_basic_error_handling()
        demonstrate_async_error_handling()
        demonstrate_multiple_device_error_handling()
        demonstrate_context_manager_error_handling()
        
        logger.info("\nAll examples executed - some errors above are expected")
        
    except Exception as e:
        logger.error(f"Fatal error in example script: {e}", exc_info=True)
        sys.exit(1)