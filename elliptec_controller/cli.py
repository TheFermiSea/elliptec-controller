#!/usr/bin/env python3
"""
Command-line interface for elliptec-controller.

This script provides basic command-line control of Elliptec rotators.
"""

import argparse
import sys
from loguru import logger
from elliptec_controller import ElliptecRotator # Changed import
import serial # For SerialException


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Elliptec Rotator Controller CLI")
    
    # Serial port
    parser.add_argument('--port', '-p', type=str, default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    
    # Rotator addresses
    parser.add_argument('--addresses', '-a', type=int, nargs='+', default=[3, 6, 8],
                        help='Rotator addresses (default: 3 6 8)')

    # Log level
    parser.add_argument('--log-level', '-l', type=str, default='INFO',
                        choices=['TRACE', 'DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
                        help='Set the logging level (default: INFO)')
    
    # Command subparsers
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Home command
    home_parser = subparsers.add_parser('home', help='Home rotators')
    home_parser.add_argument('--rotator', '-r', type=int, 
                            help='Specific rotator to home (by index, 0-based)')
    
    # Status command
    _status_parser = subparsers.add_parser('status', help='Get rotator status')

    # Move absolute command
    move_abs_parser = subparsers.add_parser('move-abs', help='Move to absolute position')
    move_abs_parser.add_argument('--rotator', '-r', type=int, required=True,
                                help='Rotator to move (by index, 0-based)')
    move_abs_parser.add_argument('--position', '-pos', type=float, required=True,
                                help='Position in degrees')
    
    # Move all command
    move_all_parser = subparsers.add_parser('move-all', help='Move all rotators')
    move_all_parser.add_argument('--positions', '-pos', type=float, nargs='+', required=True,
                                help='Positions in degrees for each rotator')
    
    # Set velocity command
    velocity_parser = subparsers.add_parser('velocity', help='Set rotator velocity')
    velocity_parser.add_argument('--rotator', '-r', type=int,
                                help='Specific rotator (by index, 0-based)')
    velocity_parser.add_argument('--value', '-v', type=int, required=True,
                                help='Velocity value (0-63)')
    
    # Info command
    info_parser = subparsers.add_parser('info', help='Get device information')
    info_parser.add_argument('--rotator', '-r', type=int,
                            help='Specific rotator (by index, 0-based)')
    
    return parser.parse_args()


def main():
    """Main CLI function."""
    args = parse_args()

    # Configure Loguru
    logger.remove() # Remove default handler
    logger.add(sys.stderr, level=args.log_level, format="<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>")

    if not args.command:
        logger.error("No command specified. Use --help for usage information.")
        return 1
    
    # Create and manage individual rotators
    rotators = []
    logger.info(f"Attempting to initialize rotators for addresses: {args.addresses} on port {args.port}")
    for i, address in enumerate(args.addresses):
        try:
            rotator_name = f"RotatorCLI-{address}"
            # auto_home=False because CLI will explicitly call home or other commands.
            # Initialization involves get_device_info.
            rot = ElliptecRotator(port=args.port, motor_address=address, name=rotator_name, auto_home=False)
            
            # Manually call get_device_info after successful port opening
            # And log its success or failure.
            logger.info(f"Attempting to get device info for {rotator_name} (Address: {address})...")
            device_info = rot.get_device_info() # Uses loguru internally
            if not device_info or device_info.get("type") in ["Unknown", "Error"]:
                logger.warning(f"Could not retrieve valid device info for {rotator_name}. Stored info: {device_info}")
            else:
                logger.info(f"Successfully initialized {rotator_name}. Device Info: {device_info}")
            rotators.append(rot)
        except serial.SerialException as e:
            logger.error(f"Serial error initializing rotator at address {address} on port {args.port}: {e}")
            # Optionally decide if one failure should stop all, or try to continue with others.
            # For now, let's try to initialize others.
        except Exception as e:
            logger.error(f"Error initializing rotator at address {address}: {e}", exc_info=True)

    if not rotators:
        logger.error("No rotators were successfully initialized. Exiting.")
        return 1

    try:
        # Process commands
        if args.command == 'home':
            if args.rotator is not None: # Specific rotator by index
                if args.rotator < 0 or args.rotator >= len(rotators):
                    logger.error(f"Error: Invalid rotator index {args.rotator}. Available: 0-{len(rotators)-1}")
                    return 1
                logger.info(f"Homing rotator {args.rotator} (Address: {rotators[args.rotator].physical_address})...")
                rotators[args.rotator].home(wait=True)
            else: # All rotators
                logger.info("Homing all successfully initialized rotators...")
                for i, r in enumerate(rotators):
                    logger.info(f"Homing rotator {i} (Address: {r.physical_address})...")
                    r.home(wait=True)
            logger.info("Homing complete for specified rotators.")
            
        elif args.command == 'status':
            logger.info("Rotator Status:")
            for i, r in enumerate(rotators):
                status = r.get_status()
                is_ready = r.is_ready()
                position = r.update_position() # Get current position
                pos_str = f"{position:.2f} deg" if position is not None else "Unknown"
                logger.info(f"  Rotator {i} (Name: {r.name}, Address: {r.physical_address}): Status {status}, Ready: {is_ready}, Position: {pos_str}")
            
        elif args.command == 'move-abs':
            if args.rotator < 0 or args.rotator >= len(rotators):
                logger.error(f"Error: Invalid rotator index {args.rotator}. Available: 0-{len(rotators)-1}")
                return 1
            r = rotators[args.rotator]
            logger.info(f"Moving rotator {args.rotator} (Address: {r.physical_address}) to {args.position} degrees...")
            r.move_absolute(args.position, wait=True)
            logger.info("Move complete.")
            
        elif args.command == 'move-all':
            if len(args.positions) != len(rotators):
                logger.error(f"Number of positions ({len(args.positions)}) must match number of initialized rotators ({len(rotators)})")
                return 1
            logger.info(f"Moving all rotators to specified positions...")
            for i, r in enumerate(rotators):
                target_pos = args.positions[i]
                logger.info(f"Moving rotator {i} (Address: {r.physical_address}) to {target_pos} degrees...")
                r.move_absolute(target_pos, wait=True)
            logger.info("All moves complete.")
            
        elif args.command == 'velocity':
            if args.rotator is not None: # Specific rotator by index
                if args.rotator < 0 or args.rotator >= len(rotators):
                    logger.error(f"Error: Invalid rotator index {args.rotator}. Available: 0-{len(rotators)-1}")
                    return 1
                r = rotators[args.rotator]
                logger.info(f"Setting rotator {args.rotator} (Address: {r.physical_address}) velocity to {args.value}...")
                r.set_velocity(args.value)
            else: # All rotators
                logger.info(f"Setting all rotator velocities to {args.value}...")
                for i, r in enumerate(rotators):
                    logger.info(f"Setting rotator {i} (Address: {r.physical_address}) velocity to {args.value}...")
                    r.set_velocity(args.value)
            logger.info("Velocity set for specified rotators.")
            
        elif args.command == 'info':
            if args.rotator is not None: # Specific rotator by index
                if args.rotator < 0 or args.rotator >= len(rotators):
                    logger.error(f"Error: Invalid rotator index {args.rotator}. Available: 0-{len(rotators)-1}")
                    return 1
                r = rotators[args.rotator]
                logger.info(f"Getting info for rotator {args.rotator} (Address: {r.physical_address})...")
                info = r.get_device_info()
                logger.info(f"Device information for rotator {args.rotator} (Name: {r.name}, Address: {r.physical_address}):")
                for key, value in info.items():
                    logger.info(f"  {key}: {value}")
            else: # All rotators
                logger.info("Getting info for all successfully initialized rotators...")
                for i, r in enumerate(rotators):
                    logger.info(f"Device information for rotator {i} (Name: {r.name}, Address: {r.physical_address}):")
                    info = r.get_device_info()
                    for key, value in info.items():
                        logger.info(f"  {key}: {value}")
        
    except Exception as e:
        logger.error(f"Error during operation: {e}", exc_info=True)
        return 1
    
    finally:
        # Close serial port for each rotator if it was opened by the class
        # The ElliptecRotator class should handle closing its own serial port upon deletion
        # if it opened it. If a shared serial object were used, it would need explicit closing.
        logger.info("CLI operations finished. Rotator serial ports will be closed by their destructors if opened by the class.")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())