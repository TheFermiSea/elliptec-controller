# Hardware Documentation

This document provides information about working with specific Thorlabs Elliptec hardware.

## Supported Devices

The following Thorlabs Elliptec devices are supported:

- ELL6 (Mini Motorized Rotation Stage)
- ELL14 (Rotation Stage)
- ELL18 (Rotation Mount)
- Other Elliptec devices that follow the same protocol

## Device Addresses

Each Elliptec device has an address (1-31) that identifies it on the bus. The default configuration for a triple rotator setup uses:

- Address 3: First half-wave plate (HWP1)
- Address 6: Quarter-wave plate (QWP)
- Address 8: Second half-wave plate (HWP2)

To change a device's address, use the Thorlabs Elliptec Software (not provided in this package).

## Physical Connections

### USB Connection

- Connect the devices using the supplied USB cable
- For multiple devices, they can be daisy-chained using the IN/OUT ports
- Only one USB connection is needed for the chain

### Power Supply

- Use the supplied power adapter
- Multiple devices can share power through the daisy chain
- Check the voltage requirements for your specific model

## Movement Characteristics

### Resolution and Range

- ELL6: 0.44° / step, 360° continuous rotation
- ELL14: 1.8° / step, 360° continuous rotation
- Other models: Refer to device documentation

### Velocity Control

- Range: 0-63 (manufacturer scale)
- Recommended range: 20-50 for most applications
- Higher velocities may reduce accuracy
- Lower velocities increase positioning accuracy

### Homing

- Required after power-up
- Uses internal reference position
- ~10-20 seconds typical duration
- Wait for completion before other commands

## Best Practices

### Initialization

1. Connect power first, then USB
2. Allow devices to stabilize (~5 seconds)
3. Home all devices before use
4. Verify ready status before operations

### Operation

1. Use appropriate velocities for your application
2. Monitor device status during movements
3. Implement error handling for timeouts
4. Use the stop command for emergency stops

### Multiple Devices

1. Assign unique addresses
2. Verify each device individually
3. Test communication with each device
4. Consider power requirements

## Error Recovery

### Common Issues

1. **Device Not Responding**
   - Check USB connection
   - Verify power supply
   - Reset device if necessary

2. **Position Errors**
   - Home the device
   - Check for mechanical obstructions
   - Verify velocity settings

3. **Communication Errors**
   - Check USB connections
   - Verify device addresses
   - Reset USB connection

### Reset Procedure

1. Power off the device
2. Disconnect USB
3. Wait 10 seconds
4. Reconnect power
5. Reconnect USB
6. Re-home device

## Performance Optimization

### Speed vs. Accuracy

- Higher velocities (40-63):
  - Faster movements
  - Reduced accuracy
  - Suitable for coarse adjustments

- Lower velocities (1-20):
  - Slower movements
  - Better accuracy
  - Use for fine positioning

### Temperature Considerations

- Allow warm-up time (~15 minutes)
- Monitor for heat buildup
- Provide adequate ventilation
- Consider ambient temperature

## Maintenance

### Regular Checks

1. Clean optical surfaces
2. Check cable connections
3. Verify smooth rotation
4. Test homing accuracy

### Preventive Measures

1. Keep area dust-free
2. Avoid cable stress
3. Use appropriate velocities
4. Regular calibration checks

## Safety Considerations

1. Never force manual rotation
2. Keep fingers clear during operation
3. Use stop command for emergencies
4. Follow all Thorlabs safety guidelines

## Additional Resources

- [Thorlabs Technical Support](https://www.thorlabs.com/support.cfm)
- [ELL6 Documentation](https://www.thorlabs.com/thorproduct.cfm?partnumber=ELL6)
- [ELL14 Documentation](https://www.thorlabs.com/thorproduct.cfm?partnumber=ELL14)