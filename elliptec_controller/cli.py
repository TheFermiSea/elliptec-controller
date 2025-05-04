#!/usr/bin/env python3
"""
Command-line interface for elliptec-controller.

This script provides basic command-line control of Elliptec rotators.
"""

import argparse
import sys
import time
from elliptec_controller import TripleRotatorController


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Elliptec Rotator Controller CLI")
    
    # Serial port
    parser.add_argument('--port', '-p', type=str, default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    
    # Rotator addresses
    parser.add_argument('--addresses', '-a', type=int, nargs='+', default=[3, 6, 8],
                        help='Rotator addresses (default: 3 6 8)')
    
    # Command subparsers
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Home command
    home_parser = subparsers.add_parser('home', help='Home rotators')
    home_parser.add_argument('--rotator', '-r', type=int, 
                            help='Specific rotator to home (by index, 0-based)')
    
    # Status command
    status_parser = subparsers.add_parser('status', help='Get rotator status')
    
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
    
    if not args.command:
        print("Error: No command specified. Use --help for usage information.")
        return 1
    
    # Create controller
    try:
        controller = TripleRotatorController(
            port=args.port,
            addresses=args.addresses
        )
    except Exception as e:
        print(f"Error initializing controller: {e}")
        return 1
    
    try:
        # Process commands
        if args.command == 'home':
            if args.rotator is not None:
                if args.rotator < 0 or args.rotator >= len(controller.rotators):
                    print(f"Error: Invalid rotator index {args.rotator}")
                    return 1
                print(f"Homing rotator {args.rotator}...")
                controller.rotators[args.rotator].home(wait=True)
            else:
                print("Homing all rotators...")
                controller.home_all(wait=True)
            print("Homing complete")
            
        elif args.command == 'status':
            print("Rotator Status:")
            for i, rotator in enumerate(controller.rotators):
                status = rotator.get_status()
                is_ready = rotator.is_ready()
                print(f"  Rotator {i} (address {rotator.address}): Status {status}, Ready: {is_ready}")
            
        elif args.command == 'move-abs':
            if args.rotator < 0 or args.rotator >= len(controller.rotators):
                print(f"Error: Invalid rotator index {args.rotator}")
                return 1
            print(f"Moving rotator {args.rotator} to {args.position} degrees...")
            controller.rotators[args.rotator].move_absolute(args.position, wait=True)
            print("Move complete")
            
        elif args.command == 'move-all':
            if len(args.positions) != len(controller.rotators):
                print(f"Error: Number of positions ({len(args.positions)}) must match number of rotators ({len(controller.rotators)})")
                return 1
            print(f"Moving all rotators to {args.positions} degrees...")
            controller.move_all_absolute(args.positions, wait=True)
            print("Move complete")
            
        elif args.command == 'velocity':
            if args.rotator is not None:
                if args.rotator < 0 or args.rotator >= len(controller.rotators):
                    print(f"Error: Invalid rotator index {args.rotator}")
                    return 1
                print(f"Setting rotator {args.rotator} velocity to {args.value}...")
                controller.rotators[args.rotator].set_velocity(args.value)
            else:
                print(f"Setting all rotator velocities to {args.value}...")
                controller.set_all_velocities(args.value)
            print("Velocity set")
            
        elif args.command == 'info':
            if args.rotator is not None:
                if args.rotator < 0 or args.rotator >= len(controller.rotators):
                    print(f"Error: Invalid rotator index {args.rotator}")
                    return 1
                print(f"Getting info for rotator {args.rotator}...")
                info = controller.rotators[args.rotator].get_device_info()
                print(f"Device information for rotator {args.rotator}:")
                for key, value in info.items():
                    print(f"  {key}: {value}")
            else:
                print("Getting info for all rotators...")
                for i, rotator in enumerate(controller.rotators):
                    info = rotator.get_device_info()
                    print(f"Device information for rotator {i}:")
                    for key, value in info.items():
                        print(f"  {key}: {value}")
        
    except Exception as e:
        print(f"Error during operation: {e}")
        return 1
    
    finally:
        # Always close the controller
        controller.close()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())