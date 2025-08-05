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

## Cloud Statistics & Monitoring

### Comprehensive Device Telemetry

The system exposes extensive real-time statistics via Particle Cloud variables for remote monitoring and diagnostics:

#### **System Health Variables**

```cpp
Particle.variable("system_uptime", systemUptime);           // Milliseconds since boot
Particle.variable("free_memory", System.freeMemory());      // Available RAM
Particle.variable("system_status", systemStatus, STRING);   // Current status
Particle.variable("last_error", lastError, STRING);         // Last error message
```

#### **CAN Communication Statistics**

```cpp
Particle.variable("can_msgs_received", canMessagesReceived);     // Total CAN RX
Particle.variable("can_msgs_sent", canMessagesSent);           // Total CAN TX
Particle.variable("can_errors_total", canErrorMonitor.totalErrors);
Particle.variable("can_recovery_count", canRecoveryCount);     // Number of recoveries
Particle.variable("can_consecutive_errors", canErrorMonitor.consecutiveErrors);
```

#### **MQTT Communication Statistics**

```cpp
Particle.variable("mqtt_msgs_received", mqttMessagesReceived);  // MQTT commands received
Particle.variable("mqtt_msgs_sent", mqttMessagesSent);         // MQTT publishes sent
Particle.variable("mqtt_fail_count", getMQTTFailCount());      // Failed publishes
Particle.variable("last_heartbeat", lastHeartbeat);           // Last heartbeat time
```

#### **Port Activity & Health**

```cpp
Particle.variable("active_ports", activePorts);              // Responsive ports (within 2 min)
Particle.variable("docked_ports", dockedPorts);             // Ports with vehicles present
// Dynamic port response tracking (1 to MAX_PORTS)
Particle.variable("port1_last_ping", lastPingTimes[1]);     // Last response from port 1
Particle.variable("port2_last_ping", lastPingTimes[2]);     // Last response from port 2
// ... continues for all ports
```

### Enhanced MQTT Health Monitoring

#### **Connection-Based Health**

The MQTT health monitoring has been redesigned for IoT devices that may not receive messages for hours:

- **Health Criteria**: Based on connection state and publish success, not message frequency
- **No False Alarms**: Hours without messages is considered normal for IoT devices
- **Proactive Detection**: Monitors actual connection health vs connection flags

#### **Automatic Heartbeat System**

```cpp
// Sends "H,0,1" every hour automatically with retry logic
const unsigned long MQTT_HEARTBEAT_INTERVAL = 60 * 60 * 1000; // 1 hour
const unsigned long MQTT_HEARTBEAT_RETRY_INTERVAL = 5 * 1000; // 5 seconds
const int MQTT_HEARTBEAT_MAX_FAILURES = 5;
```

### Port Response Tracking

#### **Response-Based Monitoring**

Port ping times track actual **responses FROM ports**, not requests TO ports:

```cpp
// Updated when IoT RECEIVES response from port (not when sending request)
void processCANMessage(can_frame msg) {
    if (parsedMsg.sourcePort >= 1 && parsedMsg.sourcePort <= MAX_PORTS) {
        updateLastPingTime(parsedMsg.sourcePort); // Tracks bidirectional communication
    }
}
```

#### **Active vs Docked Ports**

- **Active Ports**: Ports that have responded within 2 minutes (communication working)
- **Docked Ports**: Ports that have vehicles physically present
- **Health Indicator**: `active_ports == docked_ports` means all docked vehicles are responsive

### Debug Commands

Enhanced serial debug interface for real-time diagnostics:

```
MQTT_STATUS     - Complete MQTT connection diagnostics
MQTT_RECONNECT  - Force immediate reconnection
MQTT_HEARTBEAT  - Send test heartbeat
PORT_STATS      - Show port response statistics with active/docked status
SYSTEM_STATUS   - Overall system health
HELP           - Show available commands
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
hardwareWatchdog =
      new ApplicationWatchdog(20000, hardwareWatchdogHandler, 1536);

void hardwareWatchdogHandler()
{
  System.reset(RESET_NO_WAIT);
}
```

### **Enhanced RX Overflow Recovery**

#### **Escalating Recovery System**

The system now includes sophisticated RX buffer overflow handling:

```cpp
// Tracks RX overflow occurrences and effectiveness of buffer clearing
struct CANErrorMonitor {
    int rxOverflowCount;
    unsigned long firstRxOverflowTime;
    unsigned long lastRxOverflowClear;
}
```

#### **Automatic System Recovery**

- **Level 1**: Clear RX buffers (standard approach)
- **Level 2**: Track overflow frequency and persistence
- **Level 3**: Force system restart if buffer clearing fails

#### **Restart Triggers**

The system will automatically restart if:

- 5+ overflows in 30 seconds AND buffer clearing isn't helping
- 3+ overflows in 5+ seconds with recurring overflow within 1 second of clearing
- Persistent overflow despite repeated buffer clearing attempts

### **Testing & Validation**

#### **Error Condition Testing**

1. **Watchdog Test** - Disable `feedWatchdog()` to verify automatic reset
2. **Error Recovery Test** - Send messages to disconnected ports
3. **Corruption Test** - Monitor detection of invalid CAN messages
4. **Load Test** - Verify stability under heavy CAN traffic
5. **RX Overflow Test** - Verify automatic restart on persistent overflow

#### **Recovery Validation**

- Recovery completes within 5 seconds
- Normal operations resume after error clearance
- No memory leaks during recovery cycles
- Proper state cleanup between recovery attempts
- RX overflow recovery triggers within 12 seconds of detection

### **Cloud Monitoring Benefits**

#### **Real-Time Diagnostics**

- **Remote Monitoring**: All statistics accessible via Particle Cloud API
- **Historical Tracking**: Cloud variables provide trend analysis
- **Proactive Alerts**: Set up notifications based on error counts or response times
- **Performance Metrics**: Monitor CAN traffic, MQTT performance, and port responsiveness

#### **Troubleshooting Capabilities**

```
// Example cloud variable queries
port4_last_ping: 45000    → Port 4 hasn't responded in 45 seconds (problem!)
active_ports: 2           → Only 2 ports responding
docked_ports: 4           → 4 vehicles present
can_recovery_count: 12    → Multiple CAN recoveries (investigate hardware)
mqtt_fail_count: 3       → MQTT publish issues
```

#### **Health Monitoring Scenarios**

- **Healthy System**: `active_ports == docked_ports`, low error counts
- **Communication Issues**: `active_ports < docked_ports`, old ping times
- **Hardware Problems**: High `can_recovery_count`, frequent errors
- **Network Issues**: High `mqtt_fail_count`, old `last_heartbeat`

## Future Enhancements

This reorganization, safety system, and comprehensive monitoring enables:

1. **Unit Testing** - Each module can be tested independently
2. **Mock Hardware** - Hardware interfaces can be mocked for testing
3. **Feature Flags** - Easy addition of feature toggles
4. **Advanced Logging** - Structured error logging and analytics
5. **OTA Updates** - Modular structure supports partial updates
6. **Performance Monitoring** - Real-time metrics and diagnostics via cloud
7. **Predictive Maintenance** - Error pattern analysis and trend monitoring
8. **Remote Diagnostics** - Cloud-based health monitoring and troubleshooting
9. **Dashboard Integration** - Grafana, custom dashboards using Particle API
10. **Automated Alerting** - Notifications based on cloud variable thresholds
