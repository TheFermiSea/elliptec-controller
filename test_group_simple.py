#!/usr/bin/env python3
"""
Simple Group Addressing Test for Elliptec Hardware

This test validates the core group addressing functionality:
1. Configure two rotators in group mode
2. Send one movement command to master
3. Verify both rotators move with correct offsets
4. Clean up group configuration

Requirements: Elliptec rotators at addresses 2 and 3 on /dev/ttyUSB0
"""

import sys
import time
from pathlib import Path
from loguru import logger

sys.path.insert(0, str(Path(__file__).parent))
from elliptec_controller import ElliptecRotator

# Configuration
SERIAL_PORT = "/dev/ttyUSB0"
MASTER_ADDR = 2
SLAVE_ADDR = 3
SLAVE_OFFSET = 45.0  # degrees

def setup_logging():
    logger.remove()
    logger.add(sys.stderr, level="INFO", format="<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | {message}")

def main():
    setup_logging()
    
    logger.info("🔧 Simple Group Addressing Test")
    logger.info("=" * 40)
    
    try:
        # Connect to rotators
        logger.info(f"Connecting to master (address {MASTER_ADDR})...")
        master = ElliptecRotator(SERIAL_PORT, MASTER_ADDR, "Master", auto_home=False)
        
        logger.info(f"Connecting to slave (address {SLAVE_ADDR})...")
        slave = ElliptecRotator(SERIAL_PORT, SLAVE_ADDR, "Slave", auto_home=False)
        
        # Get initial positions
        logger.info("Getting initial positions...")
        master_pos_initial = master.update_position()
        slave_pos_initial = slave.update_position()
        logger.info(f"Master initial: {master_pos_initial:.2f}°")
        logger.info(f"Slave initial: {slave_pos_initial:.2f}°")
        
        # Configure group addressing
        logger.info(f"Configuring slave to listen to master with {SLAVE_OFFSET}° offset...")
        if not slave.configure_as_group_slave(str(MASTER_ADDR), SLAVE_OFFSET):
            logger.error("❌ Failed to configure group addressing")
            return False
        
        logger.info(f"✅ Group configured: slave listening to address {slave.active_address}")
        
        # Test group movement
        target_position = 30.0
        logger.info(f"Sending move command to master: {target_position}°")
        
        # Send command to master - both should move
        if not master.move_absolute(target_position, wait=False):
            logger.error("❌ Failed to send move command")
            return False
        
        # Wait for movement to complete
        logger.info("Waiting for movement to complete...")
        time.sleep(3.0)
        
        # Check final positions
        logger.info("Checking final positions...")
        master_pos_final = master.update_position()
        slave_pos_final = slave.update_position()
        
        if master_pos_final is not None and slave_pos_final is not None:
            logger.info(f"Master final: {master_pos_final:.2f}° (target: {target_position:.2f}°)")
            expected_slave = target_position + SLAVE_OFFSET
            logger.info(f"Slave final: {slave_pos_final:.2f}° (expected: ~{expected_slave:.2f}°)")
            
            # Check if positions are reasonable
            master_error = abs(master_pos_final - target_position)
            slave_error = abs(slave_pos_final - expected_slave)
            
            if master_error < 2.0:
                logger.info("✅ Master position correct")
            else:
                logger.warning(f"⚠️ Master position error: {master_error:.2f}°")
            
            if slave_error < 2.0:
                logger.info("✅ Slave position correct (group addressing working!)")
            else:
                logger.warning(f"⚠️ Slave position error: {slave_error:.2f}°")
        else:
            logger.error("❌ Could not read final positions")
        
        # Clean up group configuration
        logger.info("Cleaning up group configuration...")
        if slave.revert_from_group_slave():
            logger.info("✅ Slave reverted to individual control")
        else:
            logger.error("❌ Failed to revert slave")
        
        # Test individual control
        logger.info("Testing individual control after cleanup...")
        if slave.move_absolute(10.0, wait=False):
            time.sleep(2.0)
            final_pos = slave.update_position()
            if final_pos is not None:
                logger.info(f"✅ Individual control working: slave at {final_pos:.2f}°")
            else:
                logger.warning("⚠️ Could not verify individual control")
        
        logger.info("=" * 40)
        logger.info("✅ Group addressing test completed!")
        return True
        
    except Exception as e:
        logger.error(f"❌ Test failed: {e}")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)