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

- **`can.h/can.cpp`** - CAN bus communication with enhanced error monitoring
- **`lights.h/lights.cpp`** - LED/ring light control
- **`utils.h/utils.cpp`** - Utility functions

### System Safety & Monitoring

- **CAN Error Monitoring** - Comprehensive error detection and recovery system
- **Watchdog System** - Hardware watchdog with automatic device reset
- **Health Monitor Thread** - Independent monitoring for system freeze detection
- **Emergency Recovery** - Multi-layer safety mechanisms

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

### CAN Bus Error Handling

#### **Comprehensive Error Detection**

- **Consecutive Error Tracking** - Monitors patterns of CAN failures
- **Corruption Detection** - Catches invalid CAN messages (negative port IDs, oversized frames)
- **ERROR_A/ERROR_F Handling** - Special detection for MCP2515 controller corruption
- **Per-Port Failure Tracking** - Isolates problematic ports to prevent system-wide failures

#### **Multi-Layer Recovery System**

- **Immediate Recovery Trigger** - Activates after 3 consecutive errors (reduced from 5)
- **Port Polling Suspension** - Stops operations during error cascades
- **Automatic CAN Controller Reset** - Full MCP2515 reconfiguration
- **Emergency Device Reset** - Forces system restart if recovery fails

#### **Smart Recovery Management**

- **Recovery Attempt Limiting** - Maximum 3 attempts before device reset
- **Automatic Re-enablement** - Resumes operations when errors clear
- **Recovery State Tracking** - Prevents infinite recovery loops

### System Health & Safety

#### **Hardware Watchdog System**

- **30-Second Timeout** - Automatic device reset if system becomes unresponsive
- **Main Loop Monitoring** - Detects frozen main thread
- **Continuous Feed Mechanism** - Regular watchdog feeding during normal operation
- **Error State Protection** - Maintains watchdog even during CAN errors

#### **Independent Health Monitor**

- **Separate Monitor Thread** - Runs independently from main loop
- **Multiple Safety Triggers**:
  - 10+ consecutive CAN errors → Emergency reset
  - Recovery stuck for 10+ seconds → Emergency reset
  - Main loop frozen for 20+ seconds → Emergency reset
  - No successful operations for 60 seconds → Emergency reset
- **Real-time Status Logging** - Continuous health reporting during error conditions

#### **System Monitoring**

- Memory monitoring with leak detection
- Uptime tracking and reset reason logging
- Periodic health reports with error statistics
- Connection state monitoring (MQTT, Particle Cloud)

## Error Handling Implementation

### **Critical Safety Features**

#### **Preventing System Freeze**

```cpp
// Example error cascade prevention
if (canErrorMonitor.consecutiveErrors >= 3) {
    Serial.printlnf("CRITICAL: Immediate CAN recovery needed");
    can_recovery_needed = true;
    pollingDisabled = true;  // Stop operations immediately
    return;
}
```

#### **Emergency Recovery Process**

```cpp
void performCANRecovery() {
    Serial.printlnf("=== PERFORMING EMERGENCY CAN RECOVERY ===");

    // Stop all operations
    CAN_ERROR = true;
    can_recovery_needed = false;

    // Reset MCP2515 controller
    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    // Clear error state
    canErrorMonitor.consecutiveErrors = 0;
    CAN_ERROR = false;
}
```

#### **Independent Safety Monitor**

```cpp
void canHealthMonitorThread() {
    while (true) {
        // Emergency reset triggers
        if (canErrorMonitor.consecutiveErrors >= 10) {
            emergencyReset("Excessive consecutive CAN errors");
        }

        if (millis() - last_watchdog_feed > 20000) {
            emergencyReset("Main loop freeze detected");
        }

        delay(1000); // Check every second
    }
}
```

### **Testing & Validation**

#### **Error Condition Testing**

1. **Watchdog Test** - Disable `feedWatchdog()` to verify automatic reset
2. **Error Recovery Test** - Send messages to disconnected ports
3. **Corruption Test** - Monitor detection of invalid CAN messages
4. **Load Test** - Verify stability under heavy CAN traffic

#### **Recovery Validation**

- Recovery completes within 5 seconds
- Normal operations resume after error clearance
- No memory leaks during recovery cycles
- Proper state cleanup between recovery attempts

## Future Enhancements

This reorganization and safety system enables:

1. **Unit Testing** - Each module can be tested independently
2. **Mock Hardware** - Hardware interfaces can be mocked for testing
3. **Feature Flags** - Easy addition of feature toggles
4. **Advanced Logging** - Structured error logging and analytics
5. **OTA Updates** - Modular structure supports partial updates
6. **Performance Monitoring** - Real-time metrics and diagnostics
7. **Predictive Maintenance** - Error pattern analysis
8. **Remote Diagnostics** - Cloud-based health monitoring
