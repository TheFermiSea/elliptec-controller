# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.1] - 2025-06-02

### Fixed
- **Test Suite Complete**: Achieved 100% test passing rate (51/51 tests) by resolving all group controller test failures
- **Group Controller Mocking**: Fixed inconsistent mocking patterns in group controller tests with proper state simulation
- **Attribute References**: Corrected `active_group_address_char` → `group_master_address_char` attribute references in tests
- **Movement State Simulation**: Enhanced test mocks to properly simulate rotator movement states and device status
- **Import Dependencies**: Added missing `COMMAND_GET_STATUS` import to group controller tests
- **Error Message Formats**: Standardized error message format expectations in malformed response tests

### Improved
- **Test Infrastructure**: Standardized group formation mocking patterns across all group controller tests
- **Mock Reliability**: Implemented consistent side effects that accurately simulate hardware behavior
- **Test Coverage**: All elliptec controller functionality now has reliable test coverage
- **Developer Experience**: Fixed test failures that were blocking development workflow

### Technical Details
- Fixed 15+ failing group controller tests through systematic mocking pattern corrections
- Implemented proper `configure_as_group_slave` side effects that update rotator state
- Added `get_status` mocking for movement state verification
- Corrected test assumptions about internal state vs. hardware command success/failure
- Enhanced group formation and disbanding test logic for accurate behavior simulation

## [0.3.0] - 2025-06-02

### Added
- **Device Status Constants**: Added `STATUS_READY`, `STATUS_HOMING`, `STATUS_MOVING` constants for consistent status checking
- **Motor Status Enum**: Added `MOTOR_STATUS` enum with `MOTOR_ACTIVE` and `HOMING` bitmask values for precise device state detection
- **Real-time Status Checking**: Enhanced `is_moving` property to query actual device status rather than rely on internal state
- **Group Status Methods**: Added `get_group_status()` method to `ElliptecGroupController` for comprehensive group status monitoring

### Changed
- **⚠️ BREAKING**: `is_moving` is now a read-only property that queries device status in real-time instead of a simple boolean attribute
- **⚠️ BREAKING**: Direct assignment to `is_moving` is no longer supported - use internal `_is_moving_state` for internal tracking
- **Status System**: Replaced string literal status codes ("00", "09", "01") with named constants throughout codebase
- **Device State Logic**: Improved motor activity detection using bit masking for more reliable status interpretation
- **Group Controller**: Updated group operations to use status constants and improved state management

### Fixed
- **Status Accuracy**: Fixed race conditions where `is_moving` could be out of sync with actual device state
- **Device Communication**: Improved reliability of status checking with better error handling and timeout management
- **Group Synchronization**: Enhanced group controller status tracking for more reliable multi-device operations

### Technical Details
- Internal state tracking moved to private `_is_moving_state` attribute
- `is_moving` property now performs real-time device status queries with bit-level status analysis
- Status constants ensure consistent interpretation across all device communication
- Backward compatibility maintained for all public API methods except direct `is_moving` assignment

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
- **Hardware Validation Scripts**: Added real hardware testing tools for validation
- **UV Environment Support**: Full compatibility with modern uv package management

### Changed
- **Python Version Requirement**: Updated minimum Python version from 3.6 to 3.8
- **Logging System**: Migrated from debug flags to Loguru-based logging system
- **Configuration Management**: Improved device configuration and parameter handling
- **Position Conversion**: Enhanced accuracy in degrees-to-hex conversion using device-specific parameters
- **Group Synchronization**: Improved group addressing with configurable offsets
- **Documentation**: Comprehensive rewrite of all documentation with better examples
- **Build System**: Modernized with Hatch-based pyproject.toml configuration

### Fixed
- **Device Info Parsing**: Fixed parsing of device information responses
- **Position Accuracy**: Corrected position conversion calculations for different device types
- **Serial Buffer Management**: Improved serial port buffer handling and cleanup
- **Timeout Handling**: Better handling of communication timeouts and retries
- **Memory Leaks**: Fixed potential memory leaks in serial communication
- **Environment Compatibility**: Resolved pixi/conda-forge incompatibilities with uv migration

### Removed
- **Debug Parameters**: Removed debug flags in favor of Loguru logging configuration
- **Legacy Code**: Cleaned up deprecated methods and unused imports
- **Redundant Dependencies**: Removed unnecessary dependencies like ptpython
- **Build Artifacts**: Cleaned up setup.py, requirements.txt in favor of modern pyproject.toml

### Security
- **Input Validation**: Enhanced validation of input parameters and device responses
- **Error Exposure**: Reduced exposure of internal errors in public APIs

### Hardware Validation
- **✅ Individual Control**: 23/23 tests passing - Complete validation on real Elliptec devices
- **✅ Group Addressing**: Hardware validated with 3-rotator synchronized movement testing
- **✅ Position Accuracy**: Sub-degree precision confirmed in real-world testing
- **✅ System Integration**: Validated in μRASHG optical control systems
- **✅ Environment Compatibility**: Confirmed working with uv package management

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