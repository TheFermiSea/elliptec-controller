#!/usr/bin/env python3
"""
Hardware Test for Elliptec Group Addressing

This script tests the actual group addressing functionality with real Elliptec hardware.
It verifies that multiple rotators can be synchronized and move together with offsets.

Requirements:
- Multiple Elliptec rotators connected to the same serial port
- Rotators at addresses 2, 3, and 8 (as configured in urashg system)
- Serial port access permissions

Usage:
    python test_group_hardware.py

The test will:
1. Connect to individual rotators
2. Configure group addressing with offsets
3. Perform synchronized movements
4. Verify final positions
5. Clean up group configuration
"""

import sys
import time
from pathlib import Path
from typing import Dict, List
from loguru import logger

# Add the package to path for testing
sys.path.insert(0, str(Path(__file__).parent))

from elliptec_controller import ElliptecRotator

# Configuration
SERIAL_PORT = "/dev/ttyUSB0"
ROTATOR_ADDRESSES = [2, 3, 8]
ROTATOR_NAMES = {
    2: "HWP",      # Half-wave plate
    3: "QWP",      # Quarter-wave plate  
    8: "Analyzer"  # Analyzer
}

# Test configuration
MASTER_ADDRESS = 2  # HWP will be the master
SLAVE_OFFSETS = {
    3: 30.0,   # QWP offset: 30 degrees
    8: -45.0   # Analyzer offset: -45 degrees
}

# Movement test positions
TEST_POSITIONS = [0.0, 45.0, 90.0, 135.0]

def setup_logging():
    """Configure logging for the test."""
    logger.remove()
    logger.add(sys.stderr, level="INFO", format="<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | {message}")

def test_individual_connections() -> Dict[int, ElliptecRotator]:
    """Test individual connections to all rotators."""
    logger.info("Testing individual rotator connections...")
    rotators = {}
    
    for addr in ROTATOR_ADDRESSES:
        try:
            logger.info(f"Connecting to {ROTATOR_NAMES[addr]} at address {addr}...")
            rotator = ElliptecRotator(
                port=SERIAL_PORT,
                motor_address=addr,
                name=ROTATOR_NAMES[addr],
                auto_home=False  # Skip auto-homing for faster testing
            )
            
            # Test basic communication
            device_info = rotator.get_device_info()
            if device_info:
                logger.info(f"  ✅ {ROTATOR_NAMES[addr]}: Serial {device_info.get('serial_number', 'Unknown')}")
                rotators[addr] = rotator
            else:
                logger.error(f"  ❌ {ROTATOR_NAMES[addr]}: Failed to get device info")
                return {}
                
        except Exception as e:
            logger.error(f"  ❌ {ROTATOR_NAMES[addr]}: Connection failed - {e}")
            return {}
    
    logger.info(f"✅ Successfully connected to {len(rotators)} rotators")
    return rotators

def home_all_rotators(rotators: Dict[int, ElliptecRotator]) -> bool:
    """Home all rotators to establish reference positions."""
    logger.info("Homing all rotators...")
    
    for addr, rotator in rotators.items():
        logger.info(f"  Homing {rotator.name}...")
        if not rotator.home(wait=True):
            logger.error(f"  ❌ Failed to home {rotator.name}")
            return False
        logger.info(f"  ✅ {rotator.name} homed successfully")
    
    # Verify all are at home position (should be 0.0 degrees)
    for addr, rotator in rotators.items():
        position = rotator.update_position()
        if position is not None:
            logger.info(f"  {rotator.name} position after homing: {position:.2f}°")
        else:
            logger.warning(f"  Could not verify {rotator.name} position")
    
    return True

def test_group_configuration(rotators: Dict[int, ElliptecRotator]) -> bool:
    """Configure group addressing with the specified offsets."""
    logger.info("Configuring group addressing...")
    
    master = rotators[MASTER_ADDRESS]
    logger.info(f"Master: {master.name} (address {MASTER_ADDRESS})")
    
    # Configure slaves to listen to master's address
    for addr, rotator in rotators.items():
        if addr != MASTER_ADDRESS:
            offset = SLAVE_OFFSETS.get(addr, 0.0)
            logger.info(f"  Configuring {rotator.name} as slave with {offset}° offset...")
            
            if rotator.configure_as_group_slave(str(MASTER_ADDRESS), offset):
                logger.info(f"  ✅ {rotator.name} configured successfully")
            else:
                logger.error(f"  ❌ Failed to configure {rotator.name}")
                return False
    
    # Verify group configuration
    logger.info("Verifying group configuration...")
    for addr, rotator in rotators.items():
        if addr != MASTER_ADDRESS:
            logger.info(f"  {rotator.name}: listening to address {rotator.active_address}, is_slave={rotator.is_slave_in_group}")
    
    return True

def test_synchronized_movements(rotators: Dict[int, ElliptecRotator]) -> bool:
    """Test synchronized movements with group addressing."""
    logger.info("Testing synchronized movements...")
    
    master = rotators[MASTER_ADDRESS]
    success = True
    
    for target_pos in TEST_POSITIONS:
        logger.info(f"Moving group to {target_pos}° (master command)...")
        
        # Send move command to master - all rotators should move
        if not master.move_absolute(target_pos, wait=True):
            logger.error(f"  ❌ Failed to move to {target_pos}°")
            success = False
            continue
        
        # Wait a bit for movements to complete
        time.sleep(1.0)
        
        # Check final positions
        logger.info("  Final positions:")
        for addr, rotator in rotators.items():
            position = rotator.update_position()
            if position is not None:
                if addr == MASTER_ADDRESS:
                    expected = target_pos
                    logger.info(f"    {rotator.name}: {position:.2f}° (expected: {expected:.2f}°)")
                else:
                    # For slaves, the position should include the offset
                    expected = target_pos + SLAVE_OFFSETS.get(addr, 0.0)
                    logger.info(f"    {rotator.name}: {position:.2f}° (expected: ~{expected:.2f}°)")
                    
                    # Check if position is reasonable (within 5 degrees)
                    if abs(position - expected) > 5.0:
                        logger.warning(f"    ⚠️ {rotator.name} position may be incorrect")
            else:
                logger.error(f"    ❌ Could not get {rotator.name} position")
                success = False
        
        time.sleep(0.5)  # Brief pause between movements
    
    return success

def cleanup_group_configuration(rotators: Dict[int, ElliptecRotator]) -> bool:
    """Revert all rotators from group mode."""
    logger.info("Cleaning up group configuration...")
    
    success = True
    for addr, rotator in rotators.items():
        if addr != MASTER_ADDRESS and rotator.is_slave_in_group:
            logger.info(f"  Reverting {rotator.name} from group mode...")
            if rotator.revert_from_group_slave():
                logger.info(f"  ✅ {rotator.name} reverted successfully")
            else:
                logger.error(f"  ❌ Failed to revert {rotator.name}")
                success = False
    
    return success

def test_individual_control_after_cleanup(rotators: Dict[int, ElliptecRotator]) -> bool:
    """Verify individual control works after group cleanup."""
    logger.info("Testing individual control after cleanup...")
    
    # Try to move each rotator individually
    for addr, rotator in rotators.items():
        test_pos = 15.0  # Small test movement
        logger.info(f"  Moving {rotator.name} individually to {test_pos}°...")
        
        if rotator.move_absolute(test_pos, wait=True):
            position = rotator.update_position()
            if position is not None:
                logger.info(f"    ✅ {rotator.name} at {position:.2f}°")
            else:
                logger.warning(f"    ⚠️ Could not verify {rotator.name} position")
        else:
            logger.error(f"    ❌ Failed to move {rotator.name}")
            return False
    
    return True

def main():
    """Main test sequence."""
    setup_logging()
    
    logger.info("🔧 Elliptec Group Addressing Hardware Test")
    logger.info("=" * 50)
    
    try:
        # Step 1: Connect to all rotators
        rotators = test_individual_connections()
        if not rotators:
            logger.error("❌ Failed to connect to rotators. Exiting.")
            return False
        
        # Step 2: Home all rotators
        if not home_all_rotators(rotators):
            logger.error("❌ Failed to home rotators. Exiting.")
            return False
        
        # Step 3: Configure group addressing
        if not test_group_configuration(rotators):
            logger.error("❌ Failed to configure group addressing. Exiting.")
            return False
        
        # Step 4: Test synchronized movements
        if not test_synchronized_movements(rotators):
            logger.error("❌ Group movement tests failed.")
            # Continue to cleanup even if movements failed
        
        # Step 5: Cleanup group configuration
        if not cleanup_group_configuration(rotators):
            logger.error("❌ Failed to cleanup group configuration.")
        
        # Step 6: Verify individual control
        if not test_individual_control_after_cleanup(rotators):
            logger.error("❌ Individual control test failed after cleanup.")
        
        logger.info("=" * 50)
        logger.info("✅ Group addressing hardware test completed!")
        logger.info("Check the log output above for detailed results.")
        
        return True
        
    except KeyboardInterrupt:
        logger.warning("Test interrupted by user")
        return False
    except Exception as e:
        logger.error(f"Unexpected error during test: {e}")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)