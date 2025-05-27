# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2024-05-27

### Added
- **Loguru Integration**: Replaced print statements with comprehensive Loguru logging
- **Group Controller**: Added `ElliptecGroupController` class for managing multiple synchronized rotators
- **Enhanced Error Handling**: Improved error handling with detailed logging and recovery mechanisms
- **Device-Specific Calibration**: Automatic detection and use of device-specific pulse counts and parameters
- **Command Line Interface**: Added CLI tool `elliptec-controller` for basic operations
- **Comprehensive Testing**: Added pytest-based test suite with mock serial communication
- **Type Hints**: Added type annotations throughout the codebase
- **Thread Safety**: Enhanced thread-safe operations with proper locking mechanisms

### Changed
- **Python Version Requirement**: Updated minimum Python version from 3.6 to 3.8
- **Logging System**: Migrated from debug flags to Loguru-based logging system
- **Configuration Management**: Improved device configuration and parameter handling
- **Position Conversion**: Enhanced accuracy in degrees-to-hex conversion using device-specific parameters
- **Group Synchronization**: Improved group addressing with configurable offsets
- **Documentation**: Comprehensive rewrite of all documentation with better examples

### Fixed
- **Device Info Parsing**: Fixed parsing of device information responses
- **Position Accuracy**: Corrected position conversion calculations for different device types
- **Serial Buffer Management**: Improved serial port buffer handling and cleanup
- **Timeout Handling**: Better handling of communication timeouts and retries
- **Memory Leaks**: Fixed potential memory leaks in serial communication

### Removed
- **Debug Parameters**: Removed debug flags in favor of Loguru logging configuration
- **Legacy Code**: Cleaned up deprecated methods and unused imports
- **Redundant Dependencies**: Removed unnecessary dependencies like ptpython

### Security
- **Input Validation**: Enhanced validation of input parameters and device responses
- **Error Exposure**: Reduced exposure of internal errors in public APIs

## [0.1.0] - 2024-05-08

### Added
- Initial release of elliptec-controller package
- Basic support for Thorlabs Elliptec rotation stages (ELL6, ELL14, ELL18)
- Individual rotator control with absolute and relative positioning
- Device information retrieval (serial number, firmware, pulse counts)
- Velocity and jog step control
- Basic group addressing for synchronized movement
- Serial communication over RS-232/USB
- Simple error handling and status reporting

### Features
- `ElliptecRotator` class for individual device control
- Support for homing, positioning, and status queries
- Basic protocol implementation following ELLx manual
- Thread-safe design for multi-threaded applications
- Simple debugging with print statements

### Dependencies
- pyserial >= 3.5 for serial communication
- Standard library modules for basic functionality

---

## Version History Summary

- **v0.2.0**: Major refactor with Loguru logging, enhanced features, and improved reliability
- **v0.1.0**: Initial release with basic Elliptec rotator control functionality

For detailed technical information, see the API documentation in the `docs/` directory.