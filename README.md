# IoT Firmware Reorganization

This document describes the reorganization of the Gen 2 IoT firmware codebase to improve maintainability, testability, and code clarity.

## Overview

The original codebase had all functionality mixed together in `main.h` and `main.ino`, making it difficult to maintain and test. The reorganization separates everything into focused modules.

## New File Structure

### Core System Files

- **`main.h`** - Core system declarations and includes
- **`main.ino`** - Main system orchestration and setup/loop functions

### Configuration Management

- **`config.h`** - All configuration constants, environment settings, build parameters
- **`config.cpp`** - Configuration validation and utility functions

### MQTT Communication

- **`mqtt.h`** - MQTT connection, topics, retry logic declarations
- **`mqtt.cpp`** - Complete MQTT implementation including connection management, message handling, and command processing

### Credential Management

- **`credentials.h`** - Credential structures and management functions
- **`credentials.cpp`** - Credential fetching, validation, and state management

### Port State Management

- **`port_state.h`** - Port state structure and management functions
- **`port_state.cpp`** - Complete port state management including getters, setters, and utilities

### Hardware Interfaces (Existing)

- **`can.h/can.cpp`** - CAN bus communication (cleaned up duplicate declarations)
- **`lights.h/lights.cpp`** - LED/ring light control
- **`utils.h/utils.cpp`** - Utility functions

## Key Benefits

### 1. **Separation of Concerns**

Each module has a single, well-defined responsibility:

- `config.*` - Configuration management
- `mqtt.*` - MQTT communication
- `credentials.*` - Authentication
- `port_state.*` - Port management
- `main.*` - System orchestration

### 2. **Improved Maintainability**

- Changes to MQTT logic to not require touching port state code
- Configuration changes are centralized
- Each module can be understood independently

### 3. **Better Testing**

- Individual modules can be tested in isolation
- Mock implementations can be created for each interface
- Unit tests can focus on specific functionality

### 4. **Cleaner Dependencies**

- Explicit includes show module relationships
- Reduced coupling between components
- Clear interface boundaries

## Migration Guide

### From Old Structure

```cpp
// Old: Everything in main.h
#include "main.h"
// Access everything directly
credentialsFetched = true;
BROKER_CONNECTED = false;
```

### To New Structure

```cpp
// New: Focused includes
#include "credentials.h"
#include "mqtt.h"

// Use proper functions
if (areCredentialsValid()) {
    // ...
}
if (isMQTTConnected()) {
    // ...
}
```

## System Flow

### 1. **Initialization** (`setup()`)

```
initializeSystem()
├── initializeConfig()
├── initializePorts()
├── initializeCredentials()
└── initializeMQTT()

initializeHardware()
├── Setup lights
├── Initialize SPI
└── Configure CAN bus

initializeParticle()
├── Connect to cloud
├── Register functions
└── Log reset reason
```

### 2. **Main Loop** (`loop()`)

```
handleSystemLoop()
├── Check for CAN errors
├── Check credential failures
├── Handle connection states
├── handleMQTTClientLoop()
├── handleCredentials()
└── updateSystemStatus()
```

## Configuration Management

### Environment Configuration

All environment-specific settings are now centralized in `config.h`:

- MQTT URLs
- Credential endpoints
- Timeouts and retry intervals
- Build version information

### Runtime Configuration

Configuration validation and utility functions in `config.cpp`:

- `getCurrentEnvironment()` - Returns environment string
- `getBuildInfo()` - Returns build information

## MQTT System

### Connection Management

- Automatic reconnection with exponential backoff
- Connection state monitoring
- Graceful error handling

### Command Processing

Structured command processing in `processMQTTCommand()`:

- Charge commands (`C`)
- Unlock commands (`U`)
- Heartbeat commands (`H`)
- Version requests (`V`)
- Temperature requests (`T`)
- Emergency exit (`E`)

### Status Monitoring

- `getMQTTStatus()` - Human-readable status
- `isMQTTConnected()` - Connection state
- `shouldRetryMQTT()` - Retry logic

## Credential Management

### Secure Handling

- Structured credential storage
- Validation functions
- Retry logic with limits

### State Management

- `areCredentialsValid()` - Validation check
- `shouldRetryCredentials()` - Retry timing
- `getCredentialsStatus()` - Human-readable status

## Port State Management

### Structured Data

Each port has a complete state structure with:

- Connection status (docked, charging, secured)
- Vehicle information (VIN, validation status)
- Command flags and timeouts
- Hardware readings (voltage, current, temperature)

### Safe Operations

- Port number validation
- Safe string copying
- State query functions
- Bulk operations (reset, clear flags)

## Error Handling

### CAN Bus Errors

- Structured error reporting
- Error state management
- Visual indicators (blinking LEDs)

### System Health

- Memory monitoring
- Uptime tracking
- Periodic health reports

## Future Enhancements

This reorganization enables:

1. **Unit Testing** - Each module can be tested independently
2. **Mock Hardware** - Hardware interfaces can be mocked for testing
3. **Feature Flags** - Easy addition of feature toggles
4. **Logging System** - Structured logging can be added
5. **OTA Updates** - Modular structure supports partial updates
6. **Performance Monitoring** - Easy to add metrics per module
