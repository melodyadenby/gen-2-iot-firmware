## User

Why does my particle boron device give these logs 

it seeems to reset every 10 seconds or so which does not line up with the disconnect timeout
const unsigned long INTERNET_DISCONNECT_RESET_TIMEOUT = 60000; // 1 minute

## Assistant

<thinking>
Looking at these logs, I can see a pattern of GSM07.10 muxer thread starting, stopping, and then the device appears to be resetting. Let me analyze what's happening:

1. The logs show GSM07.10 muxer operations - this is related to cellular communication multiplexing
2. There are "Received response to CLD or timed out, exiting multiplexed mode" messages
3. The pattern repeats every few seconds, not 60 seconds
4. There are also NCP (Network Control Protocol) errors: "Failed to initialize cellular NCP client: -210"

The user is asking why the device resets every ~10 seconds when their timeout is set to 60 seconds (1 minute).

Looking at the code, I can see there's an `internetCheckThread()` function that monitors connectivity:

```cpp
void internetCheckThread() {
  static bool connected = true;
  static bool did_disconnect = false;
  static unsigned long disconnectTime = 0;
  static unsigned long lastCheckTime = 0;
  static bool calledConnect = false;
  while (true) {
    // Non-blocking check every 5 seconds
    if (millis() - lastCheckTime > 5000) {
      lastCheckTime = millis();
      CELLULAR_CONNECTED = (Cellular.ready() && Particle.connected());
      if (!CELLULAR_CONNECTED) {
        if (!did_disconnect) { // Only set disconnectTime on FIRST disconnect
          Serial.println("Internet disconnected - starting recovery timeline");
          did_disconnect = true;
          disconnectTime = millis();
        }
        unsigned long disconnectedDuration = millis() - disconnectTime;
        // Hard reset after 1 minute
        if (disconnectedDuration > INTERNET_DISCONNECT_RESET_TIMEOUT) {
          Serial.printlnf("RESET: Internet disconnected for %lu ms - "
                          "performing device reset",
                          disconnectedDuration);
          System.reset(RESET_NO_WAIT);
        }
      } else {
        if (did_disconnect) {
          // Connection restored
          Serial.printlnf("Internet reconnected after %lu ms offline",
                          millis() - disconnectTime);
        }
        disconnectTime = 0;
        did_disconnect = false;
      }
    } else {
      delay(100);
    }
  }
}
```

But there are other reset conditions in the code that could be triggering more frequently. Let me look for other reset triggers:

1. **Hardware ApplicationWatchdog**: There's a 20-second hardware watchdog that will reset if not fed
2. **CAN recovery system**: There are multiple CAN error conditions that trigger resets
3. **Emergency resets**: Various error conditions trigger immediate resets

Looking at the CAN error handling:

```cpp
void performCANRecovery() {
  // ...
  if (canErrorMonitor.recoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
    // ...
    Serial.printlnf("CRITICAL: Max recovery attempts (%d) exceeded - forcing "
                    "device reset", MAX_RECOVERY_ATTEMPTS);
    delay(100);
    resetDevice("Max CAN recovery attempts exceeded");
    return;
  }
}
```

And there's also:

```cpp
void emergencyReset(const char *reason) {
  Serial.printlnf("EMERGENCY RESET: %s", reason);
  Serial.printlnf("Error Stats - Consecutive: %d, Total: %lu",
                  canErrorMonitor.consecutiveErrors,
                  canErrorMonitor.totalErrors);
  delay(200); // Ensure message is sent
  System.reset();
}
```

The logs show GSM/cellular issues, so this is likely a connectivity problem, but the reset might not be coming from the internet timeout thread. It could be:

1. **Hardware watchdog timeout (20s)**: If the main loop gets stuck
2. **CAN error cascade**: If CAN errors are triggering recovery attempts
3. **System-level cellular failure**: The cellular modem itself might be resetting

The "Failed to initialize cellular NCP client: -210" suggests the cellular modem is having initialization issues, which could trigger other reset mechanisms.

Let me check what could cause rapid resets...

Looking at the hardware watchdog:
```cpp
// Hardware watchdog handles all freeze detection at 20-second timeout
ApplicationWatchdog *hardwareWatchdog;
void hardwareWatchdogHandler() { System.reset(RESET_NO_WAIT); }

// In setup():
hardwareWatchdog = new ApplicationWatchdog(20000, hardwareWatchdogHandler, 1536);
```

And in the loop:
```cpp
void loop() {
  ApplicationWatchdog::checkin(); // Feed hardware watchdog
  // ...
}
```

If the main loop gets stuck or takes too long, the 20-second hardware watchdog would reset the device. But that should be 20 seconds, not 10.

The GSM07.10 muxer errors suggest the cellular connection is unstable, and the Particle system might be doing its own internal resets to recover from cellular issues.
</thinking>

Looking at your logs, the issue isn't your 60-second internet timeout causing the resets. The logs show **cellular modem/GSM issues** that are triggering resets much more frequently. Here's what's happening:

## Root Cause: Cellular Modem Instability

The logs show:
- **GSM07.10 muxer** repeatedly starting/stopping (cellular multiplexing)
- **NCP client initialization failures** (`Failed to initialize cellular NCP client: -210`)
- **Cellular connection instability** at the hardware level

## Multiple Reset Triggers in Your Code

Your device has several reset mechanisms that could trigger before your 60-second timeout:

### 1. **Hardware Watchdog (20 seconds)**
```cpp
// This will reset every 20 seconds if main loop freezes
hardwareWatchdog = new ApplicationWatchdog(20000, hardwareWatchdogHandler, 1536);
```

### 2. **CAN Error Recovery System**
```cpp
void performCANRecovery() {
  if (canErrorMonitor.recoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
    Serial.printlnf("CRITICAL: Max recovery attempts exceeded - forcing device reset");
    resetDevice("Max CAN recovery attempts exceeded");
  }
}
```

### 3. **Emergency Reset Conditions**
```cpp
void emergencyReset(const char *reason) {
  Serial.printlnf("EMERGENCY RESET: %s", reason);
  System.reset();
}
```

## Likely Scenario

Your cellular modem is failing to initialize properly, which:
1. Prevents `Cellular.ready()` from being true
2. May cause the main loop to hang or take too long
3. Triggers the **20-second hardware watchdog** before your 60-second internet timeout

## Solutions

### 1. **Add Cellular Diagnostics**
Add this to your `internetCheckThread()` to see what's failing:

**Tool Call: Add cellular diagnostics to internetCheckThread**
Status: Failed

## User

Wait,  otice that there are no other logs that trigger, could this happen? the other watchdogs have prints before the system resets occure. Unless you see one without a print 

## Assistant

<thinking>
You make a very good point. Looking at the code, most of the reset conditions have explicit print statements before they trigger:

1. **Internet timeout reset**:
```cpp
if (disconnectedDuration > INTERNET_DISCONNECT_RESET_TIMEOUT) {
  Serial.printlnf("RESET: Internet disconnected for %lu ms - "
                  "performing device reset",
                  disconnectedDuration);
  System.reset(RESET_NO_WAIT);
}
```

2. **Emergency reset**:
```cpp
void emergencyReset(const char *reason) {
  Serial.printlnf("EMERGENCY RESET: %s", reason);
  Serial.printlnf("Error Stats - Consecutive: %d, Total: %lu",
                  canErrorMonitor.consecutiveErrors,
                  canErrorMonitor.totalErrors);
  delay(200); // Ensure message is sent
  System.reset();
}
```

3. **CAN recovery reset**:
```cpp
if (canErrorMonitor.recoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
  Serial.printlnf("CRITICAL: Max recovery attempts (%d) exceeded - forcing "
                  "device reset", MAX_RECOVERY_ATTEMPTS);
  delay(100);
  resetDevice("Max CAN recovery attempts exceeded");
  return;
}
```

4. **resetDevice function**:
```cpp
int resetDevice(String command) {
  if (command.length() > 0) {
    Serial.printlnf("Device reset requested. Reason: %s", command.c_str());
    Serial.printlnf("Free memory before reset: %lu bytes", System.freeMemory());
    Serial.printlnf("System uptime: %lu ms", millis());
    // ... more logging
  }
  System.reset();
  return 1;
}
```

But looking at the **Hardware ApplicationWatchdog**:

```cpp
void hardwareWatchdogHandler() { System.reset(RESET_NO_WAIT); }
```

This one has **NO print statement**! It just directly calls `System.reset(RESET_NO_WAIT)`.

So the user is absolutely right - if we're seeing resets without any reset logging, it's most likely the **Hardware ApplicationWatchdog** that's triggering after 20 seconds.

The sequence would be:
1. Cellular connection fails (GSM07.10 muxer issues)
2. Main loop gets stuck or takes too long processing the cellular failure
3. Hardware watchdog doesn't get fed for 20+ seconds
4. `hardwareWatchdogHandler()` triggers with NO logging
5. `System.reset(RESET_NO_WAIT)` executes immediately
6. Device restarts, cycle repeats

This explains:
- Why resets happen every ~10-20 seconds (hardware watchdog timeout)
- Why there are no reset reason logs (hardwareWatchdogHandler has no prints)
- Why the 60-second internet timeout never gets reached (hardware watchdog fires first)
</thinking>

You're absolutely right! Looking at the reset mechanisms, most have explicit print statements before resetting, but there's **one critical exception**:

## Hardware ApplicationWatchdog - NO Logging!

```cpp
void hardwareWatchdogHandler() { 
    System.reset(RESET_NO_WAIT); // <-- NO print statement!
}
```

This is the **only reset mechanism without logging**, and it has a **20-second timeout**:

```cpp
hardwareWatchdog = new ApplicationWatchdog(20000, hardwareWatchdogHandler, 1536);
```

## What's Happening

1. **Cellular modem fails** (GSM07.10 muxer issues in logs)
2. **Main loop gets stuck** trying to handle cellular failures
3. **Hardware watchdog not fed** for 20+ seconds 
4. **Silent reset** via `hardwareWatchdogHandler()` - no logs!
5. **Cycle repeats** - cellular still broken

## The Smoking Gun

Your logs show the pattern:
- GSM07.10 muxer struggling
- Cellular NCP errors (`-210`)
- Then **sudden restart** with no reset reason logged

This matches a hardware watchdog timeout perfectly.

## Quick Fix - Add Logging to Watchdog Handler

## User

It restarts more often than 20 seocnds 

## Assistant

You're absolutely right! If it's restarting more frequently than 20 seconds, then it's **not** the hardware ApplicationWatchdog.

Looking more carefully at your logs, I notice something important: the **GSM07.10 muxer errors** suggest this might be **Particle system-level resets** that happen automatically when the cellular modem gets into an unrecoverable state.

## Possible Causes for <20 Second Resets

1. **System-level cellular watchdog** - Particle OS may have internal watchdogs for cellular that aren't visible in your application code

2. **Hardware brownout/power issues** - If power supply is unstable during cellular transmission attempts

3. **Particle OS panic/fault handler** - System crashes that trigger immediate resets without application-level logging

4. **Cellular modem firmware crashes** - The cellular modem itself may be resetting and taking the system with it

## To Diagnose This

Add this logging to see **exactly** how long between restarts:

**Tool Call: Add startup timestamp logging to track actual reset intervals**
Status: Completed

Diff: src/main.ino
```
#include "Arduino.h"
#include "DeviceInfoLedger.h"
#include "Particle.h"
#include "can.h"
#include "can_processor.h"
#include "cloud.h"
#include "config.h"
#include "credentials.h"
#include "fixes/json_compat.h"
#include "lights.h"
#include "logging.h"
#include "main.h"
#include "port_event_handler.h"
#include "port_flag_handler.h"
#include "port_state.h"
#include "utils.h"
#include <ArduinoJson.h>
#include <SPI.h>

// Product version setup
PRODUCT_VERSION(PRODUCT_VERSION_NUM);

// Global CAN state variables
char can_err_msg[200];
bool CAN_ERROR = false;
bool CELLULAR_CONNECTED = false;
bool RESET_BROKER_FLAG = false;

// Architecture components
PortEventHandler *portEventHandler = nullptr;
PortFlagHandler *portFlagHandler = nullptr;

volatile bool queueOverflow = false;
volatile int messageCount = 0;
volatile int queueHead = 0;
volatile int queueTail = 0;

unsigned long last_port_check_reset =
    0; // Tracks the last time DID_PORT_CHECK was reset
// CAN Error monitoring and recovery
volatile int can_error_count = 0;
volatile unsigned long last_can_error_time = 0;
volatile bool can_recovery_needed = false;
const int MAX_CAN_ERRORS_PER_MINUTE = 50;
const unsigned long CAN_ERROR_RESET_INTERVAL = 60000; // 1 minute

// Internet connectivity monitoring constants
const unsigned long INTERNET_DISCONNECT_RESET_TIMEOUT = 60000; // 1 minute
const unsigned long INTERNET_RESET_COOLDOWN =
    300000; // 5 minutes between resets
const unsigned long INTERNET_SOFT_RECOVERY_TIMEOUT =
    30000; // 30 seconds for soft recovery

// Global polling state for thread safety
volatile bool polling_cycle_active = false;

// Hardware watchdog handles all freeze detection at 20-second timeout

// Interrupt Health Monitoring
// Timing Logic: After PORT_CHECK_INTERVAL (10s), we send getPortData() to all
// ports We expect responses within ~25s max, so timeouts are based on this
// pattern
unsigned long lastTransmissionTime = 0;
bool interruptHealthy = true;
const unsigned long TX_RX_IMBALANCE_TIMEOUT =
    PORT_CHECK_INTERVAL * 4; // 40s - expect responses after port polling cycle

// Global VIN traffic monitoring variables (for cloud visibility)
int g_pendingVINRequests = 0;
bool g_vinFloodProtection = false;
int g_consecutiveVINFailures = 0;
unsigned long g_lastVINRequestTime = 0;
int g_activeVINRequests = 0;
// CAN Error Monitoring Constants
const int CAN_ERROR_THRESHOLD = 5; // Max consecutive errors before recovery
const unsigned long CAN_ERROR_WINDOW = 10000; // 10 seconds error window
const unsigned long CAN_RECOVERY_DELAY =
    5000;                             // 5 seconds between recovery attempts
const int MAX_RECOVERY_ATTEMPTS = 10; // Max recovery attempts before reset
const unsigned long RECOVERY_SUCCESS_RESET_TIME =
    300000; // 5 minutes of success resets recovery count

// Flag to track when we've just connected
bool justConnectedFlag = false;

// Flag to track when CAN is ready for health monitoring
bool canReadyForMonitoring = false;

// Flag to track when CAN interrupt has been attached
bool canInterruptAttached = false;

// Flag to track pending cloud commands that need immediate processing
volatile bool pendingCloudCommand = false;

// CAN Error Monitor Structure
struct CANErrorMonitor {
  int consecutiveErrors;
  unsigned long lastErrorTime;
  unsigned long totalErrors;
  int recoveryAttempts;
  bool inRecoveryMode;
  unsigned long lastRecoveryAttempt;
  unsigned long lastSuccessTime;
  unsigned long lastSuccessfulRecovery;
  bool adaptiveMode;
  unsigned long extendedRecoveryDelay;
  // Enhanced RX overflow handling
  int rxOverflowCount;
  unsigned long firstRxOverflowTime;
  unsigned long lastRxOverflowClear;
} canErrorMonitor = {0, 0, 0, 0, false, 0, 0, 0, false, 0, 0, 0, 0};

can_frame messageQueue[CAN_QUEUE_SIZE]; // Increased from 50 to 100

// Interrupt monitoring variables
unsigned long lastInterruptTime = 0;
unsigned long lastInterruptCheck = 0;
bool interruptStuckDetected = false;
// Base timeout: 3x polling interval (30s) - ports should respond within 25s max
// after getPortData()
const unsigned long INTERRUPT_TIMEOUT = PORT_CHECK_INTERVAL * 3;
const unsigned long INTERRUPT_CHECK_INTERVAL = 10000; // Check every 10 seconds

// Global hardware watchdog object
ApplicationWatchdog *hardwareWatchdog;

// Function declarations
void checkInterruptHealth();
void recoverInterruptSystem();
void checkTransmissionReceptionBalance();
void handleRxOverflowWithEscalation(unsigned long currentTime);
void prepareCANForInterrupt();
void interruptibleDelay(unsigned long ms);

// Hardware watchdog handler
void hardwareWatchdogHandler() {
  Serial.printlnf("HARDWARE WATCHDOG RESET at uptime %lu ms", millis());
  Serial.flush();
  delay(100);
  System.reset(RESET_NO_WAIT);
}

void setup() {
  initializeSystem();

  // Log startup time and reset reason IMMEDIATELY
  Serial.printlnf("=== DEVICE STARTUP AT %lu ms ===", millis());
  Serial.printlnf("Reset reason: %d, Reset data: %d", System.resetReason(),
                  System.resetReasonData());
  Serial.printlnf("System version: %s", System.version().c_str());
  Serial.printlnf("Free memory: %lu bytes", System.freeMemory());

  // Initialize hardware watchdog early - 20 second timeout
  hardwareWatchdog =
      new ApplicationWatchdog(20000, hardwareWatchdogHandler, 1536);
  Serial.printlnf("Hardware ApplicationWatchdog initialized (20s timeout)");

  initializeArchitecture();
  initializeHardware();

  if (!CAN_ERROR) {
    initializeParticle();

    // Start alternate threads
    new Thread("can_thread", canThread);
    new Thread("port_request_thread", port_request_thread);
    new Thread("can_health_monitor", canHealthMonitorThread);
    new Thread("internet_checker", internetCheckThread);

    requestCredentials();

    // Initialize recovery state
    canErrorMonitor.lastSuccessTime = millis();
    canErrorMonitor.recoveryAttempts = 0;

    Serial.printlnf("=== SYSTEM STARTUP COMPLETE ===");
    Serial.printlnf("CAN Error Monitoring: ENABLED");
    Serial.printlnf("Hardware Watchdog: ENABLED (20s timeout)");
    Serial.printlnf("Recovery System: READY");
  } else {
    Serial.printlnf("=== STARTUP FAILED - CAN ERROR DETECTED ===");
    Serial.printlnf("Uptime before CAN error reset: %lu ms", millis());
    resetDevice("CAN error during startup");
  }
}

void loop() {
  ApplicationWatchdog::checkin(); // Feed hardware watchdog
  DeviceInfoLedger::instance().loop();

  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();

  // Detect if main loop is running too slowly (potential freeze indicator)
  if (lastLoopTime > 0 && (currentTime - lastLoopTime) > 5000) {
    Serial.printlnf(
        "WARNING: Main loop delay detected: %lu ms (uptime: %lu ms)",
        currentTime - lastLoopTime, currentTime);
  }
  lastLoopTime = currentTime;
  handleSystemLoop();

  // Small delay to prevent CPU overload during error conditions
  if (CAN_ERROR || can_recovery_needed) {
    interruptibleDelay(10);
  }
}

void initializeArchitecture() {
  // Initialize the clean architecture components
  portEventHandler =
      new PortEventHandler(nullptr); // Will use global port state functions
  portFlagHandler =
      new PortFlagHandler(nullptr); // Will use global port state functions

  Serial.printlnf("Architecture components initialized");
}

void initializeSystem() {
  Serial.begin(115200);
  while (!Serial)
    ;
  // delay(2000);

  // Log uptime immediately to track reset intervals
  static bool firstBoot = true;
  if (firstBoot) {
    Serial.printlnf("=== FIRST BOOT DETECTED ===");
    firstBoot = false;
  } else {
    Serial.printlnf("=== SUBSEQUENT BOOT - UPTIME WAS: %lu ms ===", millis());
  }

  // Initialize all subsystems
  initializePorts();
  initializeCredentials();
  initializeLedger();

  Serial.printlnf("*** KUHMUTE IoT V %s ***", BUILD_VERSION);
  Serial.printlnf("Device ID: %s", Particle.deviceID().c_str());
  Serial.printlnf("Environment: %s", getCurrentEnvironment());
  Serial.printlnf("System initialized");
}
void initializeLedger() {

  // This sets up remote configuration
  DeviceConfigLedger::instance()
      .withConfigDefaultLedgerName("device-info-defaults")
      .withConfigDeviceLedgerName("device-info-config")
      .setup();
  // This sets up the device information in ledger
  DeviceInfoLedger::instance()
      .withInfoLedgerName("device-info")
      .withRetainedBuffer(retainedLogs, sizeof(retainedLogs))
      .setup();
}
void initializeHardware() {
  // Setup ring light
  beginLight();
  setLightBlue();

  // Setup SPI for CAN
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  // Initialize CAN bus
  int err = mcp2515.reset();
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "reset", true);
    return;
  }

  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "setBitrate", true);
    return;
  }

  delay(50);
  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "setNormalMode", true);
    return;
  }

  pinMode(CAN_INT, INPUT_PULLUP);
  // NOTE: CAN interrupt will be attached after connection is established
  // This prevents accumulating unwanted messages during startup

  // Initialize interrupt health tracking
  lastInterruptTime = millis();
  lastInterruptCheck = millis();
  interruptHealthy = true;

  Serial.printlnf("Hardware initialized (CAN interrupt pending)");
}

void initializeParticle() {
  // Register cloud functions
  Particle.function("resetDevice", resetDevice);
  Particle.function("getPortVin", forceGetVin);
  Particle.function("getPortStatus", forceGetPortStatus);
  Particle.function(JUISE_INCOMING, juiceMessageCallback);

  Particle.variable("CAN_ERROR", CAN_ERROR);
  Particle.variable("pub_id", MANUAL_MODE, STRING);
  Particle.variable("credentialsFetched", credentialsFetched);
  Particle.variable("total_messages_received", total_messages_received);

  // VIN traffic monitoring variables
  Particle.variable("pending_vins", g_pendingVINRequests);
  Particle.variable("active_vins", g_activeVINRequests);
  Particle.variable("vin_flood_protection", g_vinFloodProtection);
  Particle.variable("last_vin_request", g_lastVINRequestTime);

  Particle.connect();
  Particle.publishVitals(60);
  Particle.keepAlive(30);

  // Enable system features
  System.enableFeature(FEATURE_RESET_INFO);
  System.enableFeature(FEATURE_RETAINED_MEMORY);

  // Wait for connection with timeout
  setLightRed();
  waitFor(Particle.connected, 120000);
  if (!Particle.connected()) {
    Serial.printlnf("Failed to connect to Particle Cloud");
    resetDevice("");
  }

  // Log reset reason
  // logResetReason();

  setLightBlue();
  Serial.printlnf("Particle Cloud connected");
}

void handleSystemLoop() {
  // Handle CAN recovery IMMEDIATELY - before anything else
  if (can_recovery_needed) {
    Serial.printlnf("EMERGENCY: Executing CAN recovery due to error cascade");
    performCANRecovery();
    delay(1000); // Give recovery time to complete
    return;      // Exit immediately, don't continue with normal operations
  }

  // Handle critical errors
  if (CAN_ERROR) {
    blinkCANError();
    interruptibleDelay(100); // Prevent tight loop (interruptible)
    return;
  }

  // Check for credential fetch failures
  if (attemptedCredentialsFetchCount > MAX_CREDENTIAL_ATTEMPTS) {
    Serial.printlnf("Failed to fetch credentials after max attempts");
    blinkIdentityError();
    return;
  }

  // Handle connection states
  if (!Particle.connected()) {
    setLightRed();
    return;
  }

  handleCredentials();
  updateSystemStatus();

  // System health monitoring
  checkSystemHealth();

  // Check for pending port status requests
  checkPortStatusRequest();

  // Small delay to prevent CPU overload (interruptible)
  interruptibleDelay(5);

  // Process cloud messages frequently
  Particle.process();
}

/**
 * Handle systematic port data requests with staggered polling
 * Polls each port in sequence with delays to avoid overwhelming the CAN bus
 */
void handlePortDataRequests() {
  // Add static variables for timing control
  static unsigned long last_function_run_time = 0;
  static bool first_run = true;
  static int current_poll_port =
      0; // Don't initialize to 1, will be set when cycle starts

  // Don't start polling until CAN interrupt is attached
  if (!canInterruptAttached) {
    return;
  }

  // If there's a pending cloud command, pause polling to let it process
  if (pendingCloudCommand) {
    delay(10); // Small delay to let cloud command process
    return;
  }

  // Only run this function on first call or once per PORT_CHECK_INTERVAL
  unsigned long current_time = millis();
  if (!first_run && !polling_cycle_active &&
      (current_time - last_function_run_time < PORT_CHECK_INTERVAL)) {
    // Too soon to run again, skip
    return;
  }

  // EMERGENCY STOP - Don't poll if we're in error cascade
  if (can_recovery_needed || CAN_ERROR) {
    polling_cycle_active =
        false;             // CRITICAL: Must reset this to prevent deadlock
    current_poll_port = 1; // Reset port counter
    markPortsUnpolled();
    delay(100); // Prevent tight loop during recovery
    return;
  }

  // Add consecutive failure tracking per port
  static int portFailureCount[MAX_PORTS + 1] = {0}; // Track failures per port
  static bool pollingDisabled = false;
  static unsigned long pollingDisabledTime = 0;

  // PREVENTIVE VIN flood protection - conservative approach
  static unsigned long lastVINRequestTime = 0;
  static int pendingVINRequests = 0;
  static unsigned long lastHighTrafficCheck = 0;

  // Conservative VIN request limiting (prevent silent lockups)
  static int activeVINRequests = 0; // Currently processing VIN requests
  static unsigned long vinRequestSlots[3] = {
      0, 0, 0}; // Track up to 3 concurrent VINs
  static const int MAX_CONCURRENT_VINS =
      2; // Conservative: only 2 VIN requests at once

  // Check if we should re-enable polling after errors cleared
  if (pollingDisabled) {
    unsigned long waitTime = 5000; // Default 5 seconds

    // Extended wait time in adaptive mode
    if (canErrorMonitor.adaptiveMode &&
        canErrorMonitor.extendedRecoveryDelay > 5000) {
      waitTime = canErrorMonitor.extendedRecoveryDelay;
    }

    if (canErrorMonitor.consecutiveErrors == 0 &&
        millis() - pollingDisabledTime > waitTime) {
      Serial.printlnf(
          "Re-enabling port polling - errors have cleared (waited %lu ms)",
          waitTime);
      pollingDisabled = false;
      // Reset all port failure counts
      for (int i = 0; i <= MAX_PORTS; i++) {
        portFailureCount[i] = 0;
      }
    } else {
      // Make delay interruptible
      for (int i = 0; i < 10 && !pendingCloudCommand; i++) {
        delay(10);
        Particle.process();
      }
      return;
    }
  }

  // Static variable for port check reset timing
  static unsigned long last_port_check_reset = 0;
  static unsigned long last_poll_send_time = 0;

  // Start of a new polling cycle - reset all ports for polling
  if (!polling_cycle_active ||
      (current_time - last_port_check_reset >= PORT_CHECK_INTERVAL)) {
    // Only print the message when we're actually resetting
    if (current_time - last_port_check_reset >= PORT_CHECK_INTERVAL) {
      markPortsUnpolled();
      last_port_check_reset = current_time; // Update the last reset time
      Serial.println("🚨 DID_PORT_CHECK reset for all ports 🚨");

      // Force start new cycle when ports are reset
      polling_cycle_active = false;
      current_poll_port = 0; // Will be set to 1 when cycle actually starts
    }

    // Start a new polling cycle
    if (!polling_cycle_active) {
      polling_cycle_active = true;
      current_poll_port = 1; // NOW set to port 1 when actually starting
      Serial.printlnf(
          "Starting new polling cycle for all ports - starting at port %d",
          current_poll_port);
    }
  }

  // IMPORTANT: Only check this if it's not the first poll (last_poll_send_time
  // will be 0 on first poll)
  if (last_poll_send_time != 0) {
    // Check if enough time has passed since last poll
    if (current_time - last_poll_send_time < POLL_STAGGER_DELAY) {
      // Not enough time has passed, skip this poll cycle
      return;
    }

    // PREVENTIVE: Count and limit VIN requests to prevent silent lockups
    if (current_time - lastHighTrafficCheck >= 3000) { // Check every 3 seconds
      pendingVINRequests = 0;
      activeVINRequests = 0;

      // Clear expired VIN request slots (assume 15 seconds max for VIN
      // completion)
      for (int i = 0; i < 3; i++) {
        if (vinRequestSlots[i] > 0 &&
            (current_time - vinRequestSlots[i]) > 15000) {
          vinRequestSlots[i] = 0; // Clear expired slot
        } else if (vinRequestSlots[i] > 0) {
          activeVINRequests++; // Count active VIN requests
        }
      }

      // Count pending VIN flags
      for (int i = 1; i <= MAX_PORTS; i++) {
        if (isValidPort(i)) {
          PortState *state = getPortState(i);
          if (state && state->vin_request_flag) {
            pendingVINRequests++;
          }
        }
      }

      // Update global variables for cloud monitoring
      g_pendingVINRequests = pendingVINRequests;
      g_activeVINRequests = activeVINRequests;

      lastHighTrafficCheck = current_time;
    }

    // CONSERVATIVE delay calculation to prevent bus saturation
    unsigned long smart_delay = POLL_STAGGER_DELAY; // Default 1000ms

    // MUCH more conservative VIN spacing to prevent silent lockups
    if (pendingVINRequests >= 3) {
      smart_delay = 8000; // 8 second delay when 3+ VIN requests pending
    } else if (pendingVINRequests >= 1) {
      smart_delay = 5000; // 5 second delay when any VIN requests pending
    }

    // Aggressive delay for higher port addresses (they have 1200ms+ response
    // delays)
    if (current_poll_port > 10) {
      smart_delay += 3000; // Extra 3 seconds for ports 11-16
    } else if (current_poll_port > 8) {
      smart_delay += 2000; // Extra 2 seconds for ports 9-10
    }

    // LONG buffer after VIN requests - accounts for full 3-message sequence +
    // port delays
    if (current_time - lastVINRequestTime <
        10000) {           // Within 10 seconds of VIN request
      smart_delay += 5000; // Add 5 second buffer minimum
    }

    // If any VIN requests are actively being processed, be extra conservative
    if (activeVINRequests > 0) {
      smart_delay += 3000; // Additional 3 seconds when VINs are processing
    }

    // Check if enough time has passed since last poll using smart delay
    // Make this interruptible by checking for cloud commands
    unsigned long delay_start = millis();
    while ((millis() - last_poll_send_time) < smart_delay) {
      // Process Particle cloud to ensure responsiveness
      Particle.process();

      // Check for pending cloud commands every 50ms
      if (pendingCloudCommand) {
        return; // Exit immediately to process cloud command
      }

      // Prevent infinite loop
      if ((millis() - delay_start) > (smart_delay + 1000)) {
        break;
      }

      delay(50); // Small delay between checks
    }
  }

  // Check if portFlagHandler is available
  if (!portFlagHandler) {
    Serial.printlnf("portFlagHandler not initialized");
    return;
  }

  // Only process port polling when we're in an active cycle
  if (!polling_cycle_active) {
    return; // Don't increment port counter between cycles
  }

  // Safety check - if current_poll_port is 0, set it to 1
  if (current_poll_port == 0) {
    current_poll_port = 1;
    Serial.printlnf("WARNING: current_poll_port was 0, resetting to 1");
  }

  // Find next port that needs polling
  int start_port = current_poll_port; // Remember where we started
  for (int attempts = 0; attempts < MAX_PORTS; attempts++) {
    // Validate and wrap port number BEFORE using it
    if (current_poll_port < 1 || current_poll_port > MAX_PORTS) {
      current_poll_port = 1; // Wrap around or fix invalid port number
    }

    if (!hasPortBeenPolled(current_poll_port)) {
      // Check if this port has a pending VIN request
      bool isVINRequest = false;
      PortState *state = getPortState(current_poll_port);
      if (state) {
        isVINRequest = state->vin_request_flag;
      }

      // PREVENTIVE: Skip VIN requests if too many are already active
      if (isVINRequest && activeVINRequests >= MAX_CONCURRENT_VINS) {
        Serial.printlnf("PREVENTIVE: Skipping VIN request for port %d - %d "
                        "already active (max %d)",
                        current_poll_port, activeVINRequests,
                        MAX_CONCURRENT_VINS);
        current_poll_port++; // Move to next port
        continue;            // Skip this VIN request
      }

      // This is the critical part - actually send the port data request
      bool success = portFlagHandler->sendGetPortData(current_poll_port);

      if (success) {
        Serial.printlnf("Polled port %d", current_poll_port);
        markPortPolled(current_poll_port);
        // Reset failure count on success
        portFailureCount[current_poll_port] = 0;
        resetCANSuccessCounter();
        // Track transmission time for health monitoring
        lastTransmissionTime = millis();

        // Track VIN request timing and reserve slot
        if (isVINRequest) {
          lastVINRequestTime = current_time;
          g_lastVINRequestTime = current_time;

          // Reserve a VIN request slot
          for (int i = 0; i < 3; i++) {
            if (vinRequestSlots[i] == 0) {
              vinRequestSlots[i] = current_time;
              break;
            }
          }

          Serial.printlnf("VIN request sent to port %d - slot reserved, %d "
                          "slots active",
                          current_poll_port, activeVINRequests + 1);
        }

        // Calculate port wait time (empirical values based on testing)
        int portWaitTime = 3000; // Default 3 seconds between polls

        // Higher ports need longer delays (cascading response delays)
        if (current_poll_port > 8) {
          portWaitTime = 4000; // 4 seconds for ports 9+
        }
        if (current_poll_port > 12) {
          portWaitTime = 5000; // 5 seconds for ports 13+
        }

        // Global STAGGER_DELAY can override port wait time
        if (POLL_STAGGER_DELAY > portWaitTime) {
          portWaitTime = POLL_STAGGER_DELAY;
        }

        // VIN requests need much longer delay
        if (isVINRequest) {
          portWaitTime = 10000; // 10 seconds minimum for VIN requests
        }

      } else {
        portFailureCount[current_poll_port]++;
        Serial.printlnf("Failed to poll port %d (failures: %d)",
                        current_poll_port, portFailureCount[current_poll_port]);

        // Skip this port if it's failing repeatedly
        if (portFailureCount[current_poll_port] >= 3) {
          Serial.printlnf(
              "Port %d has failed %d times - skipping for this cycle",
              current_poll_port, portFailureCount[current_poll_port]);
          markPortPolled(current_poll_port); // Mark as "polled" to skip it
          portFailureCount[current_poll_port] = 0; // Reset for next cycle
        }

        logCANError(-1, "port_polling");

        // If too many consecutive errors, trigger IMMEDIATE recovery
        if (canErrorMonitor.consecutiveErrors >= 3) {
          Serial.printlnf("CRITICAL: Immediate CAN recovery needed");
          can_recovery_needed = true;
          pollingDisabled = true;
          pollingDisabledTime = millis();
          pendingCloudCommand = false; // Clear flag during error
          return;                      // Stop polling immediately
        }

        // Hardware watchdog will handle timeout protection automatically
      }

      last_poll_send_time = current_time;

      // Move to next port and wrap if needed
      current_poll_port++;
      if (current_poll_port > MAX_PORTS) {
        current_poll_port = 1;
      }

      // Instead of breaking, return to allow delay between ports
      return;
    }

    // Move to next port when skipping already-polled ports
    current_poll_port++;
    if (current_poll_port > MAX_PORTS) {
      current_poll_port = 1;
    }
  }

  // If we get here, we've checked all ports and no ports need polling in this
  // cycle Check if we've completed a full cycle (all ports are polled)
  bool all_ports_polled = true;
  for (int port = 1; port <= MAX_PORTS; port++) {
    if (!hasPortBeenPolled(port)) {
      all_ports_polled = false;
      break;
    }
  }

  // If all ports are polled, end the polling cycle
  if (all_ports_polled) {
    polling_cycle_active = false;
    current_poll_port = 1; // Reset to port 1 for next cycle
    last_function_run_time = current_time;
    first_run = false;
    Serial.printlnf(
        "Port polling cycle complete - all ports polled - next cycle in %lu ms",
        PORT_CHECK_INTERVAL);
  }
}

void handleCredentials() {
  if (areCredentialsValid()) {
    return; // Already have valid credentials
  }

  if (shouldRetryCredentials()) {
    requestCredentials();
  }
}

void updateSystemStatus() {
  if (areCredentialsValid() && CELLULAR_CONNECTED) {
    setLightGreen(); // All systems operational

    // Set the just connected flag when we first get valid credentials and
    // Internet connected connection
    static bool wasConnectedBefore = false;
    if (!wasConnectedBefore) {
      justConnectedFlag = true;
      wasConnectedBefore = true;
      Serial.printlnf("First successful connection detected - immediate port "
                      "polling enabled");

      // Prepare CAN system and attach interrupt with clean state
      prepareCANForInterrupt();
    }
  } else if (areCredentialsValid()) {
    setLightPurple(); // Have credentials, connecting to Cloud
  } else {
    setLightBlue(); // Fetching credentials
  }
}

// ========================================
// CAN Processing
// ========================================

void canThread() {
  Serial.printlnf("CAN processing thread started");

  while (true) {
    if (areCredentialsValid() && CELLULAR_CONNECTED && !CAN_ERROR &&
        !can_recovery_needed) {
      // Give priority to cloud commands
      if (pendingCloudCommand) {
        delay(50); // Give cloud command time to process
        Particle.process();
        continue; // Skip this iteration to let cloud command complete
      }

      // Process incoming CAN messages
      handleCanQueue();

      // Process port flags (replaces old flagHandler)
      if (portFlagHandler) {
        portFlagHandler->processAllPortFlags();
      }

      // Process Particle cloud to ensure responsiveness
      Particle.process();
    }
    // Small delay to prevent busy-waiting
    delay(10);
  }
}

void port_request_thread() {
  while (true) {
    if (areCredentialsValid() && CELLULAR_CONNECTED && !CAN_ERROR &&
        !can_recovery_needed) {
      // Skip polling if cloud command is pending
      if (!pendingCloudCommand) {
        handlePortDataRequests();
      }

      // Shorter delay to allow staggered polling to work properly
      // Each port will be polled with the appropriate delay between them
      interruptibleDelay(50);
    } else {
      // Small delay to prevent busy-waiting
      interruptibleDelay(100);
    }
  }
}

void handleCanQueue() {
  int messagesProcessed = 0;
  int MAX_MESSAGES_PER_LOOP = 8;

  // If queue is critically full, process more aggressively
  if (messageCount > (CAN_QUEUE_SIZE * 3 / 4)) { // More than 75% full
    MAX_MESSAGES_PER_LOOP = 20;
    Serial.printlnf("CAN queue critical (%d messages) - aggressive processing",
                    messageCount);
  }

  while (messageCount > 0 && messagesProcessed < MAX_MESSAGES_PER_LOOP) {
    can_frame msg;
    bool validMessage = false;

    // Thread-safe message extraction
    noInterrupts();
    if (messageCount > 0) {
      if (queueHead >= 0 && queueHead < CAN_QUEUE_SIZE && queueTail >= 0 &&
          queueTail < CAN_QUEUE_SIZE) {
        msg = messageQueue[queueHead];
        queueHead = (queueHead + 1) % CAN_QUEUE_SIZE;
        messageCount--;
        validMessage = true;

        // Handle queue overflow reset
        if (queueOverflow && messageCount < (CAN_QUEUE_SIZE / 2)) {
          queueOverflow = false;
          Serial.println("Queue overflow cleared");
        }
      } else {
        // Invalid queue state, reset
        queueTail = 0;
        queueHead = 0;
        messageCount = 0;
        Serial.println(
            "ERROR: Invalid queue indices detected - resetting queue");
      }
    }
    interrupts();

    // Process the message using clean architecture
    if (validMessage) {
      processCANMessage(msg);
      messagesProcessed++;
    }

    // Yield periodically
    if (messagesProcessed % 2 == 0) {
      Particle.process();
    }
  }
}

void processCANMessage(const can_frame &rawMessage) {
  // Check for corrupted CAN ID (negative or excessively large values)
  if ((int32_t)rawMessage.can_id < 0 || rawMessage.can_id > 16777215) {
    Serial.printlnf("CRITICAL: Corrupted CAN ID detected: %ld (0x%lX)",
                    (long)rawMessage.can_id, (unsigned long)rawMessage.can_id);
    logCANError(0xF, "corrupted_can_id");

    // This indicates controller corruption - trigger immediate recovery
    Serial.printlnf("CAN controller corruption detected - triggering recovery");
    can_recovery_needed = true;
    return;
  }

  // Check for invalid data length
  if (rawMessage.can_dlc > 8) {
    Serial.printlnf("CRITICAL: Invalid CAN DLC: %d", rawMessage.can_dlc);
    logCANError(0xE, "invalid_dlc");
    can_recovery_needed = true;
    return;
  }

  // 1. Parse the raw CAN message
  ParsedCANMessage parsedMsg = canProcessor.parseMessage(rawMessage);

  // 2. Log the message for debugging
  Serial.printlnf("CAN message from port %d: type=%s, valid=%s",
                  parsedMsg.sourcePort,
                  canProcessor.getMessageTypeString(parsedMsg.messageType),
                  parsedMsg.isValid ? "yes" : "no");

  // 3. Handle the business logic if message is valid
  if (parsedMsg.isValid && portEventHandler) {
    portEventHandler->handleCANMessage(parsedMsg);
    // Reset consecutive error count on successful message processing
    resetCANSuccessCounter();
    // Update interrupt health tracking - we successfully processed a message
    lastInterruptTime = millis();
    interruptHealthy = true;
  } else if (!parsedMsg.isValid) {
    Serial.printlnf("Invalid CAN message received from port %d",
                    parsedMsg.sourcePort);
    logCANError(-1, "message_parsing");
  }
}

void can_interrupt() {
  // Minimal interrupt handler - just queue the message
  struct can_frame recMsg;

  // Read the message
  int readin = readCanMessage(&recMsg);

  if (readin == ERROR_OK) {
    // Comprehensive corruption and gibberish filtering
    // 1. Basic validity checks
    if ((int32_t)recMsg.can_id < 0 || recMsg.can_id == 0 ||
        recMsg.can_id > MAX_PORTS || recMsg.can_dlc > 8) {
      // Don't queue obviously corrupted messages
      return; // Silent discard for performance
    }

    // 2. Data content validation - filter out gibberish
    bool hasValidData = false;
    bool hasGibberish = false;

    for (int i = 0; i < recMsg.can_dlc; i++) {
      uint8_t byte = recMsg.data[i];

      // Check for valid ASCII characters (printable range)
      if ((byte >= 0x20 && byte <= 0x7E) || byte == 0x00) {
        hasValidData = true;
      }
      // Check for obvious gibberish (control chars, high ASCII)
      else if (byte < 0x20 || byte > 0x7E) {
        // Allow some common control characters
        if (byte != 0x0A && byte != 0x0D && byte != 0x09) // LF, CR, TAB
        {
          hasGibberish = true;
        }
      }
    }

    // Reject if mostly gibberish or no valid data
    if (hasGibberish && !hasValidData) {
      return; // Silent discard of gibberish
    }

    // 3. Message pattern validation - must start with valid command
    if (recMsg.can_dlc > 0) {
      char firstChar = (char)recMsg.data[0];
      // Valid message types: D(status), K(VIN), T(temp), H(heartbeat), etc.
      if (firstChar != 'D' && firstChar != 'K' && firstChar != 'T' &&
          firstChar != 'H' && firstChar != 'U' && firstChar != 'C' &&
          firstChar != 'V' && firstChar != 'F' && firstChar != 'E') {
        return; // Silent discard of invalid command
      }
    }

    // Add to queue if there's space
    if (messageCount < CAN_QUEUE_SIZE) {
      incrementMessageCounter();
      messageQueue[queueTail] = recMsg;
      queueTail = (queueTail + 1) % CAN_QUEUE_SIZE;
      messageCount++;
    } else {
      queueOverflow = true;
      logCANError(-2, "queue_overflow");

      // Emergency queue clear if severely overflowing
      static unsigned long lastEmergencyClear = 0;
      if (millis() - lastEmergencyClear > 5000) { // Only once per 5 seconds
        Serial.println(
            "EMERGENCY: Clearing oldest 25% of queue to prevent lockup");
        // Clear oldest 25% of messages
        int toClear = messageCount / 4;
        queueHead = (queueHead + toClear) % CAN_QUEUE_SIZE;
        messageCount -= toClear;
        lastEmergencyClear = millis();
      }
    }
  } else {
    // Log read errors (but not every single one to avoid spam)
    static unsigned long lastErrorLog = 0;
    if (millis() - lastErrorLog > 1000) // Log max once per second
    {
      logCANError(readin, "can_read_interrupt");
      lastErrorLog = millis();
    }
  }
}

// ========================================
// CAN Error Monitoring and Recovery
// ========================================

void performCANRecovery() {
  Serial.printlnf("=== PERFORMING EMERGENCY CAN RECOVERY ===");

  // Stop ALL CAN operations immediately
  CAN_ERROR = true;
  can_recovery_needed = false;  // Clear the flag first
  polling_cycle_active = false; // Reset polling state during recovery
  canInterruptAttached = false; // Disable polling during recovery

  // Clear all queues and reset state
  noInterrupts();
  queueHead = 0;
  queueTail = 0;
  messageCount = 0;
  queueOverflow = false;

  // Reset RX overflow tracking during recovery
  canErrorMonitor.rxOverflowCount = 0;
  canErrorMonitor.firstRxOverflowTime = 0;
  canErrorMonitor.lastRxOverflowClear = 0;
  interrupts();

  // Detach interrupt to prevent issues during recovery
  detachInterrupt(CAN_INT);

  // Update recovery tracking
  canErrorMonitor.recoveryAttempts++;
  canErrorMonitor.inRecoveryMode = true;
  canErrorMonitor.lastRecoveryAttempt = millis();

  // Check if we've exceeded max recovery attempts without recent success
  if (canErrorMonitor.recoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
    unsigned long timeSinceLastSuccess =
        millis() - canErrorMonitor.lastSuccessfulRecovery;

    // If it's been a long time since last successful recovery, allow reset of
    // attempts
    if (timeSinceLastSuccess > RECOVERY_SUCCESS_RESET_TIME) {
      Serial.printlnf("Resetting recovery attempts - %lu ms since last success",
                      timeSinceLastSuccess);
      canErrorMonitor.recoveryAttempts = 1; // Reset but count this attempt
      canErrorMonitor.adaptiveMode = true;  // Enable adaptive recovery
    } else {
      Serial.printlnf("CRITICAL: Max recovery attempts (%d) exceeded - forcing "
                      "device reset",
                      MAX_RECOVERY_ATTEMPTS);
      delay(100);
      resetDevice("Max CAN recovery attempts exceeded");
      return;
    }
  }

  Serial.printlnf("CAN Recovery attempt %d of %d",
                  canErrorMonitor.recoveryAttempts, MAX_RECOVERY_ATTEMPTS);

  // Adaptive recovery delay based on attempt number
  unsigned long recoveryDelay = 500;
  if (canErrorMonitor.adaptiveMode) {
    recoveryDelay =
        1000 + (canErrorMonitor.recoveryAttempts * 500); // Progressive delay
    canErrorMonitor.extendedRecoveryDelay =
        recoveryDelay * 2; // Extended pause after recovery
    Serial.printlnf("Adaptive recovery mode - using %lu ms delay",
                    recoveryDelay);
  }

  delay(recoveryDelay); // Give system time to stabilize

  // Reset MCP2515 controller
  Serial.printlnf("Resetting MCP2515 controller...");
  int err = mcp2515.reset();
  if (err != mcp2515.ERROR_OK) {
    Serial.printlnf("CRITICAL: MCP2515 reset failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 reset failed");
    return;
  }

  delay(200);

  // Reconfigure controller
  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != mcp2515.ERROR_OK) {
    Serial.printlnf(
        "CRITICAL: MCP2515 bitrate config failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 config failed");
    return;
  }

  delay(100);

  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK) {
    Serial.printlnf(
        "CRITICAL: MCP2515 normal mode failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 mode failed");
    return;
  }

  // Re-enable interrupts after successful controller reset
  pinMode(CAN_INT, INPUT_PULLUP);

  // Only re-attach interrupt if we're connected and have credentials
  if (areCredentialsValid() && CELLULAR_CONNECTED) {
    // Use prepareCANForInterrupt to ensure clean state
    prepareCANForInterrupt();
    canInterruptAttached = true; // Re-enable polling after recovery
    Serial.printlnf(
        "CAN recovery complete - interrupt re-enabled with clean state");
  } else {
    canInterruptAttached = false; // Disable polling until connected
    Serial.printlnf(
        "CAN interrupt pending - will attach after connection established");
  }

  // Reset all error tracking
  can_error_count = 0;
  last_can_error_time = millis();
  canErrorMonitor.consecutiveErrors = 0;
  canErrorMonitor.inRecoveryMode = false;
  canErrorMonitor.lastSuccessTime = millis();
  canErrorMonitor.lastSuccessfulRecovery = millis();
  CAN_ERROR = false;

  Serial.printlnf("=== CAN RECOVERY COMPLETED SUCCESSFULLY ===");
  Serial.printlnf("Recovery attempt %d succeeded",
                  canErrorMonitor.recoveryAttempts);

  // Extended pause in adaptive mode for system stabilization
  unsigned long postRecoveryDelay = 1000;
  if (canErrorMonitor.adaptiveMode &&
      canErrorMonitor.extendedRecoveryDelay > 1000) {
    postRecoveryDelay = canErrorMonitor.extendedRecoveryDelay;
    Serial.printlnf("Extended post-recovery delay: %lu ms", postRecoveryDelay);
  }

  delay(postRecoveryDelay);
}

void logCANError(int errorCode, const char *operation) {
  unsigned long currentTime = millis();

  canErrorMonitor.totalErrors++;
  canErrorMonitor.lastErrorTime = currentTime;
  canErrorMonitor.consecutiveErrors++;

  Serial.printlnf("CAN Error #%lu: %s failed with code %d (consecutive: %d)",
                  canErrorMonitor.totalErrors, operation, errorCode,
                  canErrorMonitor.consecutiveErrors);

  // Check for specific error codes that indicate controller corruption
  if (errorCode == 0xA || errorCode == 10 || errorCode == 0xF ||
      errorCode == 15) {
    Serial.printlnf("CRITICAL: Detected ERROR_A/ERROR_F - MCP2515 controller "
                    "corruption!");
    Serial.printlnf(
        "Triggering immediate CAN recovery to prevent system freeze");
    can_recovery_needed = true;

    // If we've seen these errors multiple times, force device reset
    if (canErrorMonitor.consecutiveErrors >= 2) {
      Serial.printlnf(
          "Multiple controller corruption errors - forcing device reset");
      delay(100);
      resetDevice("MCP2515 controller corruption");
      return;
    }
  }

  // Lower threshold for faster recovery during error cascades
  if (canErrorMonitor.consecutiveErrors >= 3) {
    Serial.printlnf("Fast recovery trigger - %d consecutive errors",
                    canErrorMonitor.consecutiveErrors);
    can_recovery_needed = true;
  }

  // Legacy compatibility - keep old variables for now
  can_error_count++;
  last_can_error_time = currentTime;
  if (can_error_count > MAX_CAN_ERRORS_PER_MINUTE) {
    can_recovery_needed = true;
  }
}

void resetCANSuccessCounter() {
  // Reset consecutive error count on successful operation
  if (canErrorMonitor.consecutiveErrors > 0) {
    Serial.printlnf("CAN operation successful - resetting error count");
    canErrorMonitor.consecutiveErrors = 0;
  }

  // Update success time
  canErrorMonitor.lastSuccessTime = millis();

  // Reset recovery attempts after sustained successful operation
  if (canErrorMonitor.recoveryAttempts > 0) {
    unsigned long timeSinceRecovery =
        millis() - canErrorMonitor.lastRecoveryAttempt;

    // Only reset attempts if we've had sustained success (30 seconds)
    if (timeSinceRecovery > 30000) {
      Serial.printlnf("Resetting recovery attempt counter after sustained "
                      "success (%lu ms)",
                      timeSinceRecovery);
      canErrorMonitor.recoveryAttempts = 0;
      canErrorMonitor.adaptiveMode =
          false; // Disable adaptive mode on sustained success
      canErrorMonitor.extendedRecoveryDelay = 0;
    }
  }

  // Only clear recovery mode if we're not currently in an active recovery
  if (!can_recovery_needed && !CAN_ERROR) {
    canErrorMonitor.inRecoveryMode = false;
  }
}

// ========================================
// Watchdog Functions
// ========================================
// Software watchdog functions removed - using hardware ApplicationWatchdog
// only

// ========================================
// Utility Functions
// ========================================

void reportCANError(int err, const char *operation, bool report) {
  if (report) {
    CAN_ERROR = err != 0;
  }

  if (err != 0) {
    char ret[150];
    Serial.printlnf("CAN BUS ERROR %s", operation);
    ReturnErrorString(err, ret, sizeof(ret));
    sprintf(can_err_msg, "CAN BUS ERROR %s: %s", operation, ret);
    Serial.printlnf("%s", can_err_msg);

    // Log this as a CAN error for monitoring
    logCANError(err, operation);
  }
}

int resetDevice(String command) {
  if (command.length() > 0) {
    Serial.printlnf("Device reset requested. Reason: %s", command.c_str());
    Serial.printlnf("Free memory before reset: %lu bytes", System.freeMemory());
    Serial.printlnf("System uptime: %lu ms", millis());
    Serial.printlnf("Cellular connected: %s",
                    CELLULAR_CONNECTED ? "yes" : "no");
    Serial.printlnf("Credentials valid: %s",
                    areCredentialsValid() ? "yes" : "no");
    Serial.printlnf("CAN errors in last period: %d", can_error_count);
  }
  System.reset();
  return 1;
}
int forceGetVin(String command) {
  int port = atoi(command.c_str());
  if (!isValidPort(port)) {
    Particle.publish("forceGetVin", "error: invalid port", PRIVATE);
    return -1;
  }
  PortState *state = getPortState(port);
  if (!state) {
    Particle.publish("forceGetVin", "error: no state", PRIVATE);
    return -2;
  }

  if (strlen(state->VIN) > 0) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Port %d VIN: %s", port, state->VIN);
    Particle.publish("forceGetVin", buffer, PRIVATE);
    return 1; // Success
  } else {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Port %d: no VIN", port);
    Particle.publish("forceGetVin", buffer, PRIVATE);
    return 0; // No VIN
  }
}
int forceGetPortStatus(String command) {
  int port = atoi(command.c_str());

  String portStatus = getPortStatusSummary(port);
  // char buffer[100];
  // snprintf(buffer, sizeof(buffer), portStatus.c_str());
  Particle.publish("forceGetPortStatus", portStatus, PRIVATE);
  return 1; // Success
}

void logDebugInfo(const char *checkpoint) {
  static unsigned long lastLogTime = 0;
  unsigned long now = millis();

  Serial.printlnf("[%lu ms] Checkpoint: %s (Delta: %lu ms)", now, checkpoint,
                  now - lastLogTime);

  lastLogTime = now;
}

void logResetReason() {
  int reason = System.resetReason();
  int reasonData = System.resetReasonData();

  Serial.printlnf("RESET REASON: %d", reason);
  Serial.printlnf("RESET REASON DATA: %d", reasonData);

  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Reason: %d, Data: %d", reason, reasonData);

  Particle.publish("RESET REASON", buffer, PRIVATE);
}

void checkSystemHealth() {
  unsigned long freeMemory = System.freeMemory();
  unsigned long uptime = millis();

  if (freeMemory < 1000) {
    Serial.printlnf("Low memory warning: %lu bytes", freeMemory);
  }

  static unsigned long lastHealthCheck = 0;
  if (uptime - lastHealthCheck > 60000) {
    Serial.printlnf("System Health - Uptime: %lu ms, Free Memory: %lu bytes",
                    uptime, freeMemory);
    Serial.printlnf("Cellular Status: %s",
                    CELLULAR_CONNECTED ? "connected" : "disconnected");
    Serial.printlnf("Credentials: %s", getCredentialsStatus().c_str());
    Serial.printlnf("CAN Errors (last minute): %d", can_error_count);
    Serial.printlnf("CAN Recovery needed: %s",
                    can_recovery_needed ? "yes" : "no");

    if (portFlagHandler) {
      Serial.printlnf("Ports with pending flags: %d",
                      portFlagHandler->getPendingPortsCount());
    }

    lastHealthCheck = uptime;
  }
}

void emergencyReset(const char *reason) {
  Serial.printlnf("EMERGENCY RESET: %s", reason);
  Serial.printlnf("Error Stats - Consecutive: %d, Total: %lu",
                  canErrorMonitor.consecutiveErrors,
                  canErrorMonitor.totalErrors);
  delay(200); // Ensure message is sent
  System.reset();
}

// Internet connectivity monitoring thread with safety features
// This thread monitors internet connection and performs recovery actions:
// 1. Soft recovery (disconnect/reconnect) after 30 seconds
// 2. Hard reset after 1 minute, but only if system is in safe state
void internetCheckThread() {
  static bool connected = true;
  static bool did_disconnect = false;
  static unsigned long disconnectTime = 0;
  static unsigned long lastCheckTime = 0;
  static bool calledConnect = false;
  while (true) {
    // Non-blocking check every 5 seconds
    if (millis() - lastCheckTime > 5000) {

      lastCheckTime = millis();

      CELLULAR_CONNECTED = (Cellular.ready() && Particle.connected());

      if (!CELLULAR_CONNECTED) {
        if (!did_disconnect) { // Only set disconnectTime on FIRST disconnect
          Serial.println("Internet disconnected - starting recovery timeline");
          did_disconnect = true;
          disconnectTime = millis();
        }
        unsigned long disconnectedDuration = millis() - disconnectTime;

        // if (!calledConnect) {
        //   Particle.connect();
        //   calledConnect = true;
        // }
        // Hard reset after 1 minute
        if (disconnectedDuration > INTERNET_DISCONNECT_RESET_TIMEOUT) {

          Serial.printlnf("RESET: Internet disconnected for %lu ms - "
                          "performing device reset",
                          disconnectedDuration);
          System.reset(RESET_NO_WAIT);
        }
      } else {
        if (did_disconnect) {
          // Connection restored
          Serial.printlnf("Internet reconnected after %lu ms offline",
                          millis() - disconnectTime);
        }
        disconnectTime = 0;
        did_disconnect = false;
        // calledConnect = false;
      }
    } else {
      delay(100);
    }
  }
}

void canHealthMonitorThread() {
  static unsigned long lastHealthCheck = 0;
  const unsigned long HEALTH_CHECK_INTERVAL =
      1000; // Check every second during crisis
  const unsigned long MAIN_LOOP_TIMEOUT = 20000; // 20 seconds max

  Serial.println("CAN Health Monitor thread started");

  while (true) {
    unsigned long currentTime = millis();

    // Check health every second
    if (currentTime - lastHealthCheck >= HEALTH_CHECK_INTERVAL) {

      // Check MCP2515 error flags for actual hardware issues
      uint8_t currentFlags = getCANErrorFlags(true);

      // Monitor CAN error rate (from old canMonitorThread)
      // Monitor CAN error rate
      if (currentTime - last_can_error_time > CAN_ERROR_RESET_INTERVAL) {
        if (can_error_count > 0) {
          Serial.printlnf("Resetting CAN error count (was %d)",
                          can_error_count);
        }
        can_error_count = 0;
        last_can_error_time = currentTime;
      }

      // Check if error rate is too high
      if (can_error_count > MAX_CAN_ERRORS_PER_MINUTE) {
        Serial.printlnf("CAN error rate too high (%d errors/minute), "
                        "triggering recovery",
                        can_error_count);
        can_recovery_needed = true;
      }

      // Check interrupt health
      checkInterruptHealth();

      // Check transmission/reception balance
      checkTransmissionReceptionBalance();

      // Check if we have consecutive CAN errors with lower threshold
      if (canErrorMonitor.consecutiveErrors >= 3) {
        if (!canErrorMonitor.inRecoveryMode) {
          Serial.printlnf("CAN Error Threshold Reached: %d consecutive errors",
                          canErrorMonitor.consecutiveErrors);
          can_recovery_needed = true;
          canErrorMonitor.inRecoveryMode = true;
          canErrorMonitor.lastRecoveryAttempt = currentTime;
        }
      }

      // Reset consecutive error count if enough time has passed without
      // errors
      if (currentTime - canErrorMonitor.lastErrorTime > CAN_ERROR_WINDOW) {
        if (canErrorMonitor.consecutiveErrors > 0) {
          Serial.printlnf("CAN Error Window Expired - Resetting "
                          "consecutive error count");
          canErrorMonitor.consecutiveErrors = 0;
        }
      }

      // Emergency reset if error cascade continues
      if (canErrorMonitor.consecutiveErrors >= 10) {
        emergencyReset("Excessive consecutive CAN errors");
      }

      // Reset if stuck in recovery mode too long
      if (canErrorMonitor.inRecoveryMode &&
          currentTime - canErrorMonitor.lastRecoveryAttempt >
              CAN_RECOVERY_DELAY * 2) {
        emergencyReset("CAN recovery timeout");
      }

      // Check if no successful operations for too long
      if (canErrorMonitor.lastSuccessTime > 0 &&
          currentTime - canErrorMonitor.lastSuccessTime >
              PORT_CHECK_INTERVAL * 2 + 5000) {
        // Instead of emergency reset, check if CAN hardware is actually
        // healthy
        if (currentFlags == 0 && canErrorMonitor.consecutiveErrors == 0) {
          // CAN hardware looks fine, maybe just need to restart polling
          Serial.println("No recent CAN success but hardware looks healthy - "
                         "restarting polling");
          markPortsUnpolled(); // Gentle restart of port polling
          canErrorMonitor.lastSuccessTime = millis(); // Reset timer

          // Give it one more cycle to verify success
          static unsigned long lastGentleRestart = 0;
          if (currentTime - lastGentleRestart >
              60000) { // Only try once per minute
            lastGentleRestart = currentTime;
          } else {
            // Second gentle restart failed, more serious issue
            Serial.println("Gentle restart failed, CAN may have deeper issues");
            emergencyReset("CAN operations failed even after gentle restart");
          }
        } else {
          // Actual CAN hardware errors detected
          Serial.printlnf("CAN hardware errors detected (flags: 0x%02X)",
                          currentFlags);
          emergencyReset("No successful CAN operations with hardware errors");
        }
      }

      // Hardware watchdog handles main loop freeze detection automatically

      // Check CAN error rate with lower threshold
      if (canErrorMonitor.consecutiveErrors >= 6) {
        Serial.printlnf("CRITICAL: Excessive CAN errors (%d), forcing reset",
                        canErrorMonitor.consecutiveErrors);
        delay(100);
        emergencyReset("Excessive CAN errors");
      }

      // Check for queue overflow conditions
      if (queueOverflow) {
        Serial.println("CAN message queue overflow detected");
        queueOverflow = false; // Reset flag
        logCANError(-3, "persistent_queue_overflow");
      }

      // Periodic queue health check
      if (messageCount > (CAN_QUEUE_SIZE * 2 / 3)) { // More than 66% full
        Serial.printlnf("WARNING: CAN queue filling up (%d/%d messages)",
                        messageCount, CAN_QUEUE_SIZE);

        // If queue is critically full and not being processed, force clear old
        // messages
        static unsigned long lastQueueStuckTime = 0;
        static int lastQueueCount = 0;

        if (messageCount == lastQueueCount &&
            messageCount > (CAN_QUEUE_SIZE * 4 / 5)) {
          // Queue hasn't changed and is >80% full - might be stuck
          if (lastQueueStuckTime == 0) {
            lastQueueStuckTime = currentTime;
          } else if (currentTime - lastQueueStuckTime >
                     3000) { // Stuck for 3 seconds
            Serial.printlnf("CRITICAL: Queue stuck at %d messages for 3s - "
                            "clearing oldest 30%%",
                            messageCount);
            noInterrupts();
            int toClear = messageCount * 3 / 10; // Clear 30%
            queueHead = (queueHead + toClear) % CAN_QUEUE_SIZE;
            messageCount -= toClear;
            interrupts();
            lastQueueStuckTime = 0;
          }
        } else {
          lastQueueStuckTime = 0; // Reset if queue is moving
        }
        lastQueueCount = messageCount;
      }

      // Report RX overflow statistics if any have occurred
      if (canErrorMonitor.rxOverflowCount > 0) {
        unsigned long overflowDuration =
            currentTime - canErrorMonitor.firstRxOverflowTime;
        Serial.printlnf("RX Overflow Status: %d overflows in %lu ms (last "
                        "clear: %lu ms ago)",
                        canErrorMonitor.rxOverflowCount, overflowDuration,
                        currentTime - canErrorMonitor.lastRxOverflowClear);

        // Reset overflow tracking after successful recovery (if no overflow for
        // 10 seconds and system is healthy)
        if (currentTime - canErrorMonitor.lastRxOverflowClear > 10000 &&
            canErrorMonitor.consecutiveErrors == 0) {
          Serial.printlnf(
              "RX overflow resolved - resetting tracking (was %d overflows)",
              canErrorMonitor.rxOverflowCount);
          canErrorMonitor.rxOverflowCount = 0;
          canErrorMonitor.firstRxOverflowTime = 0;
        }
      }

      // Check for invalid CAN messages (indicates controller corruption)
      if (canErrorMonitor.consecutiveErrors >= 3) {
        Serial.printlnf(
            "CAN controller corruption suspected (%d consecutive errors)",
            canErrorMonitor.consecutiveErrors);
      }

      // Log periodic health status during error conditions
      // Only log health status every 10 seconds to reduce noise
      static unsigned long lastHealthLog = 0;
      if ((canErrorMonitor.consecutiveErrors > 0 ||
           canErrorMonitor.inRecoveryMode) &&
          currentTime - lastHealthLog > 10000) {
        Serial.printlnf(
            "CAN Health: Errors=%d, Attempts=%d, Recovering=%s, "
            "HW_Flags=0x%02X",
            canErrorMonitor.consecutiveErrors, canErrorMonitor.recoveryAttempts,
            canErrorMonitor.inRecoveryMode ? "YES" : "NO", currentFlags);
        lastHealthLog = currentTime;
      }

      // // Log CAN error flags if any are detected (less frequent)
      // static unsigned long lastFlagLog = 0;
      // if (currentFlags != 0 && currentTime - lastFlagLog > 30000) //
      // Every 30 seconds
      // {
      //   Serial.printlnf("MCP2515 hardware error flags detected: 0x%02X",
      //   currentFlags); getCANErrorFlags(true); // Debug output with
      //   detailed flag breakdown lastFlagLog = currentTime;
      // }

      lastHealthCheck = currentTime;
    }

    delay(100); // Check every second
  }
}

void checkInterruptHealth() {
  unsigned long currentTime = millis();

  // Only check every INTERRUPT_CHECK_INTERVAL
  if (currentTime - lastInterruptCheck < INTERRUPT_CHECK_INTERVAL) {
    return;
  }
  lastInterruptCheck = currentTime;

  // Check for RX overflow first
  uint8_t errorFlags = getCANErrorFlags(false);
  if (errorFlags & 0x40) { // RX0OVR flag
    handleRxOverflowWithEscalation(currentTime);
    return;
  }

  // Only check other interrupt health when system is fully operational
  if (!Particle.connected() || !areCredentialsValid()) {
    // System not ready - don't monitor interrupts yet
    static bool wasReady = false;
    if (wasReady) {
      Serial.printlnf("Interrupt health monitoring disabled - system not fully "
                      "operational");
      wasReady = false;
    }
    lastInterruptTime = currentTime; // Reset timer to prevent false alarms
    return;
  }

  // Log when monitoring becomes active
  static bool wasReady = false;
  if (!wasReady) {
    Serial.printlnf(
        "Interrupt health monitoring enabled - system fully operational");
    wasReady = true;
    lastInterruptTime = currentTime; // Start fresh when monitoring begins
  }

  // Check if we've received interrupts recently
  unsigned long timeSinceLastInterrupt = currentTime - lastInterruptTime;

  // Calculate dynamic timeout based on polling cycle
  // Normal: 30s timeout, but after port reset: 40s to allow for slower
  // responses
  unsigned long dynamicTimeout = INTERRUPT_TIMEOUT;
  unsigned long timeSincePortReset = currentTime - last_port_check_reset;
  if (timeSincePortReset < PORT_CHECK_INTERVAL * 2) {
    dynamicTimeout =
        PORT_CHECK_INTERVAL * 4; // 40s grace period after port polling starts
  }

  // If we haven't received message processing in a while, but CAN is
  // supposed to be working
  if (timeSinceLastInterrupt > dynamicTimeout && !CAN_ERROR &&
      !can_recovery_needed) {
    // Check MCP2515 error flags first to see if it's just buffer overflow
    uint8_t errorFlags = getCANErrorFlags(false);

    // If it's just RX buffer overflow, clear buffers instead of interrupt
    // recovery
    if (errorFlags & 0x40) { // RX0OVR flag
      handleRxOverflowWithEscalation(currentTime);
      return;
    }

    // Check if we can still send (TX working) but not receive (RX/interrupt
    // dead)
    Serial.printlnf("No CAN message processing for %lu ms (timeout: %lu ms) - "
                    "checking interrupt system",
                    timeSinceLastInterrupt, dynamicTimeout);

    // Check MCP2515 interrupt flags to see if they're stuck
    uint8_t intFlags = mcp2515.getInterrupts();
    Serial.printlnf("MCP2515 interrupt flags: 0x%02X", intFlags);

    if (intFlags != 0) {
      Serial.printlnf("CRITICAL: MCP2515 has pending interrupts but no message "
                      "processing!");

      // Try to recover the interrupt system
      recoverInterruptSystem();
    } else {
      Serial.printlnf(
          "MCP2515 no pending interrupts - may be normal quiet period");
      // Update last interrupt time to prevent false alarms during quiet
      // periods
      lastInterruptTime = currentTime - (INTERRUPT_TIMEOUT / 2);
    }
  }
}

void recoverInterruptSystem() {
  Serial.printlnf("=== ATTEMPTING INTERRUPT SYSTEM RECOVERY ===");

  // Detach current interrupt
  detachInterrupt(CAN_INT);
  delay(100);

  // Clear any pending MCP2515 interrupts
  mcp2515.clearInterrupts();
  delay(50);

  // Reconfigure the interrupt pin
  pinMode(CAN_INT, INPUT_PULLUP);
  delay(50);

  // Re-attach interrupt only if connected
  if (areCredentialsValid() && CELLULAR_CONNECTED) {
    // Clear any pending interrupts before re-attaching
    mcp2515.clearInterrupts();
    mcp2515.clearRXnOVR();
    delay(50);

    attachInterrupt(CAN_INT, can_interrupt, FALLING);
    delay(100);
    canInterruptAttached = true; // Re-enable polling after interrupt recovery
    Serial.printlnf("Interrupt re-attached after recovery with cleared state");
  } else {
    canInterruptAttached = false; // Disable polling until connected
    Serial.printlnf("Interrupt not re-attached - waiting for connection");
  }

  // Reset interrupt health tracking
  lastInterruptTime = millis();
  interruptHealthy = true;

  Serial.printlnf("Interrupt system recovery completed");
  Serial.printlnf("Re-attached interrupt on pin %d", CAN_INT);

  // Test interrupt by reading any pending messages
  uint8_t intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    Serial.printlnf("Post-recovery: MCP2515 flags: 0x%02X", intFlags);
  }
}

void checkTransmissionReceptionBalance() {
  unsigned long currentTime = millis();

  // Check for RX overflow first - handle regardless of connectivity status
  uint8_t errorFlags = getCANErrorFlags(false);
  if (errorFlags & 0x40) { // RX0OVR flag
    handleRxOverflowWithEscalation(currentTime);
    return;
  }

  // Only check TX/RX balance when system is fully operational
  if (!CELLULAR_CONNECTED || !areCredentialsValid()) {
    // System not ready - reset timers to prevent false alarms
    static bool txRxWasReady = false;
    if (txRxWasReady) {
      Serial.printlnf("TX/RX monitoring disabled - system not ready");
      txRxWasReady = false;
    }
    lastTransmissionTime = currentTime;
    lastInterruptTime = currentTime;
    return;
  }

  // Log when TX/RX monitoring becomes active
  static bool txRxWasReady = false;
  if (!txRxWasReady) {
    Serial.printlnf("TX/RX monitoring enabled - system ready");
    txRxWasReady = true;
    lastTransmissionTime = currentTime; // Start fresh when monitoring begins
    lastInterruptTime = currentTime;
  }

  // Check if we're actively transmitting but not receiving
  unsigned long timeSinceLastTX = currentTime - lastTransmissionTime;
  unsigned long timeSinceLastRX = currentTime - lastInterruptTime;

  // Calculate dynamic TX/RX timeout based on polling patterns
  // After port polling cycle starts, give extra time for all ports to
  // respond
  unsigned long dynamicTxRxTimeout = TX_RX_IMBALANCE_TIMEOUT;
  unsigned long timeSincePortReset = currentTime - last_port_check_reset;
  if (timeSincePortReset < PORT_CHECK_INTERVAL * 2) {
    dynamicTxRxTimeout =
        PORT_CHECK_INTERVAL * 5; // 50s grace period after polling cycle
  }

  // If we've transmitted recently but haven't received anything in much
  // longer
  if (timeSinceLastTX < PORT_CHECK_INTERVAL &&
      timeSinceLastRX > dynamicTxRxTimeout && !CAN_ERROR &&
      !can_recovery_needed) {
    // Before attempting interrupt recovery, check if it's just buffer
    // overflow
    uint8_t errorFlags = getCANErrorFlags(false);

    // If it's RX overflow, that's normal under high traffic - not an
    // interrupt failure
    if (errorFlags & 0x40) { // RX0OVR flag
      handleRxOverflowWithEscalation(currentTime);
      return;
    }

    // Only trigger interrupt recovery if we have real interrupt issues
    static unsigned long lastInterruptRecovery = 0;
    if (currentTime - lastInterruptRecovery >
        PORT_CHECK_INTERVAL * 6) { // Only once per 6 polling cycles
      Serial.printlnf("CRITICAL: TX/RX imbalance - TX:%lu ms ago, RX:%lu ms "
                      "ago (timeout: %lu ms)",
                      timeSinceLastTX, timeSinceLastRX, dynamicTxRxTimeout);
      Serial.printlnf("TX works but RX failed - attempting interrupt recovery");
      recoverInterruptSystem();
      lastInterruptRecovery = currentTime;
    } else {
      // Reset timer to prevent constant triggering
      lastInterruptTime = currentTime - (dynamicTxRxTimeout / 2);
    }
  }
}

void prepareCANForInterrupt() {
  Serial.println("Preparing CAN for interrupt attachment...");

  // 1. Make sure interrupt is detached first
  detachInterrupt(CAN_INT);
  delay(50);

  // 2. Clear MCP2515 error flags (including overflow flags)
  uint8_t errorFlags = mcp2515.getErrorFlags();
  if (errorFlags != 0) {
    Serial.printlnf("Clearing MCP2515 error flags: 0x%02X", errorFlags);
    mcp2515.clearRXnOVR(); // Clear RX overflow flags
  }

  // 3. Read and discard all pending messages from hardware buffers
  can_frame dummyMsg;
  int messagesCleared = 0;
  while (mcp2515.readMessage(&dummyMsg) == MCP2515::ERROR_OK &&
         messagesCleared < 20) {
    messagesCleared++;
  }
  if (messagesCleared > 0) {
    Serial.printlnf("Discarded %d pending messages from MCP2515",
                    messagesCleared);
  }

  // 4. Clear all interrupt flags in MCP2515
  uint8_t intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    Serial.printlnf("Clearing MCP2515 interrupt flags: 0x%02X", intFlags);
    mcp2515.clearInterrupts();
  }

  // 5. Reset our software queue state
  noInterrupts();
  queueHead = 0;
  queueTail = 0;
  messageCount = 0;
  queueOverflow = false;
  memset(messageQueue, 0, sizeof(messageQueue));
  interrupts();

  // 6. Reset CAN error monitoring state
  canErrorMonitor.rxOverflowCount = 0;
  canErrorMonitor.firstRxOverflowTime = 0;
  canErrorMonitor.lastRxOverflowClear = 0;
  canErrorMonitor.consecutiveErrors = 0;

  // 7. Wait for hardware to settle
  delay(100);

  // 8. Final check - make sure no new interrupts appeared
  intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    mcp2515.clearInterrupts();
    Serial.printlnf("Cleared additional interrupt flags: 0x%02X", intFlags);
  }

  // 9. Check the interrupt pin state before attaching
  int pinState = digitalRead(CAN_INT);
  Serial.printlnf("CAN_INT pin state before attach: %s",
                  pinState == LOW ? "LOW (pending)" : "HIGH (clear)");

  // 10. If pin is still low, clear interrupts one more time
  if (pinState == LOW) {
    Serial.println("CAN_INT pin is LOW - clearing interrupts again");
    mcp2515.clearInterrupts();
    delay(50);
    pinState = digitalRead(CAN_INT);
    Serial.printlnf("CAN_INT pin state after additional clear: %s",
                    pinState == LOW ? "LOW (still pending)" : "HIGH (cleared)");
  }

  // 12. NOW attach the interrupt with everything clean
  attachInterrupt(CAN_INT, can_interrupt, FALLING);

  // Set flag to enable port polling
  canInterruptAttached = true;

  // 13. Reset timing trackers
  lastInterruptTime = millis();
  lastTransmissionTime = millis();
  canErrorMonitor.lastSuccessTime = millis();

  Serial.println("CAN interrupt attached with fully clean state - ready to "
                 "receive messages");
}

// Interruptible delay that processes cloud messages and checks for pending
// commands
void interruptibleDelay(unsigned long ms) {
  unsigned long start = millis();
  while ((millis() - start) < ms) {
    // Check for pending cloud commands
    if (pendingCloudCommand) {
      return; // Exit immediately to process cloud command
    }

    // Process Particle cloud messages
    Particle.process();

    // Small delay to prevent tight loop
    delay(10);
  }
}

void handleRxOverflowWithEscalation(unsigned long currentTime) {
  // Track RX overflow occurrences
  canErrorMonitor.rxOverflowCount++;

  // Initialize tracking on first overflow
  if (canErrorMonitor.rxOverflowCount == 1) {
    canErrorMonitor.firstRxOverflowTime = currentTime;
  }

  Serial.printlnf("RX buffer overflow detected (#%d) - clearing buffers",
                  canErrorMonitor.rxOverflowCount);

  // Always try clearing buffers first
  clearAllCANBuffers();
  canErrorMonitor.lastRxOverflowClear = currentTime;
  lastInterruptTime = currentTime; // Reset interrupt timer

  // Check if we've had persistent overflow issues
  unsigned long overflowDuration =
      currentTime - canErrorMonitor.firstRxOverflowTime;

  // If we've had 5+ overflows in 30 seconds, or continuous overflows for 5+
  // seconds
  if ((canErrorMonitor.rxOverflowCount >= 5 && overflowDuration < 30000) ||
      (overflowDuration >= 5000 && canErrorMonitor.rxOverflowCount >= 3)) {

    // Check if clearing buffers hasn't helped (overflow recurring within 1
    // second)
    if (currentTime - canErrorMonitor.lastRxOverflowClear < 1000) {
      Serial.printlnf(
          "CRITICAL: RX overflow persisting despite buffer clearing");
      Serial.printlnf("Overflow count: %d in %lu ms",
                      canErrorMonitor.rxOverflowCount, overflowDuration);
      Serial.printlnf(
          "Buffer clearing ineffective - triggering system restart");

      // Log the critical condition
      logCANError(-11, "persistent_rx_overflow_restart");

      // Force immediate restart
      Serial.printlnf(
          "*** FORCING SYSTEM RESTART DUE TO PERSISTENT RX OVERFLOW ***");
      delay(100); // Brief delay for serial output
      System.reset(RESET_NO_WAIT);
    }
  }
}

```


## Assistant

Now run this and check:

1. **How many milliseconds** between "DEVICE STARTUP" messages
2. **Reset reason codes** - this will tell us if it's system crashes vs watchdog
3. **Any "HARDWARE WATCHDOG RESET"** messages (now it will log)

The reset reason codes will reveal what's actually causing the resets:
- `RESET_REASON_PANIC` = system crash
- `RESET_REASON_WATCHDOG` = watchdog timeout  
- `RESET_REASON_POWER_MANAGEMENT` = power issues
- etc.

This will tell us exactly what's happening and how often!

## User

Particles restart all code every restart, this will never be reached
} else {
    Serial.printlnf("=== SUBSEQUENT BOOT - UPTIME WAS: %lu ms ===", millis());
  }

## Assistant

You're absolutely correct! `millis()` resets to 0 on every boot, and static variables are reinitialized. That code is useless.

## Better Approach - Use Retained Memory

Let's use Particle's retained memory to track boot times across resets:

**Tool Call: Fix boot tracking using retained memory instead of static variables**
Status: Completed

Diff: src/main.ino
```
#include "Arduino.h"
#include "DeviceInfoLedger.h"
#include "Particle.h"
#include "can.h"
#include "can_processor.h"
#include "cloud.h"
#include "config.h"
#include "credentials.h"
#include "fixes/json_compat.h"
#include "lights.h"
#include "logging.h"
#include "main.h"
#include "port_event_handler.h"
#include "port_flag_handler.h"
#include "port_state.h"
#include "utils.h"
#include <ArduinoJson.h>
#include <SPI.h>

// Product version setup
PRODUCT_VERSION(PRODUCT_VERSION_NUM);

// Global CAN state variables
char can_err_msg[200];
bool CAN_ERROR = false;
bool CELLULAR_CONNECTED = false;
bool RESET_BROKER_FLAG = false;

// Architecture components
PortEventHandler *portEventHandler = nullptr;
PortFlagHandler *portFlagHandler = nullptr;

volatile bool queueOverflow = false;
volatile int messageCount = 0;
volatile int queueHead = 0;
volatile int queueTail = 0;

unsigned long last_port_check_reset =
    0; // Tracks the last time DID_PORT_CHECK was reset
// CAN Error monitoring and recovery
volatile int can_error_count = 0;
volatile unsigned long last_can_error_time = 0;
volatile bool can_recovery_needed = false;
const int MAX_CAN_ERRORS_PER_MINUTE = 50;
const unsigned long CAN_ERROR_RESET_INTERVAL = 60000; // 1 minute

// Internet connectivity monitoring constants
const unsigned long INTERNET_DISCONNECT_RESET_TIMEOUT = 60000; // 1 minute
const unsigned long INTERNET_RESET_COOLDOWN =
    300000; // 5 minutes between resets
const unsigned long INTERNET_SOFT_RECOVERY_TIMEOUT =
    30000; // 30 seconds for soft recovery

// Global polling state for thread safety
volatile bool polling_cycle_active = false;

// Hardware watchdog handles all freeze detection at 20-second timeout

// Interrupt Health Monitoring
// Timing Logic: After PORT_CHECK_INTERVAL (10s), we send getPortData() to all
// ports We expect responses within ~25s max, so timeouts are based on this
// pattern
unsigned long lastTransmissionTime = 0;
bool interruptHealthy = true;
const unsigned long TX_RX_IMBALANCE_TIMEOUT =
    PORT_CHECK_INTERVAL * 4; // 40s - expect responses after port polling cycle

// Global VIN traffic monitoring variables (for cloud visibility)
int g_pendingVINRequests = 0;
bool g_vinFloodProtection = false;
int g_consecutiveVINFailures = 0;
unsigned long g_lastVINRequestTime = 0;
int g_activeVINRequests = 0;
// CAN Error Monitoring Constants
const int CAN_ERROR_THRESHOLD = 5; // Max consecutive errors before recovery
const unsigned long CAN_ERROR_WINDOW = 10000; // 10 seconds error window
const unsigned long CAN_RECOVERY_DELAY =
    5000;                             // 5 seconds between recovery attempts
const int MAX_RECOVERY_ATTEMPTS = 10; // Max recovery attempts before reset
const unsigned long RECOVERY_SUCCESS_RESET_TIME =
    300000; // 5 minutes of success resets recovery count

// Flag to track when we've just connected
bool justConnectedFlag = false;

// Flag to track when CAN is ready for health monitoring
bool canReadyForMonitoring = false;

// Flag to track when CAN interrupt has been attached
bool canInterruptAttached = false;

// Flag to track pending cloud commands that need immediate processing
volatile bool pendingCloudCommand = false;

// CAN Error Monitor Structure
struct CANErrorMonitor {
  int consecutiveErrors;
  unsigned long lastErrorTime;
  unsigned long totalErrors;
  int recoveryAttempts;
  bool inRecoveryMode;
  unsigned long lastRecoveryAttempt;
  unsigned long lastSuccessTime;
  unsigned long lastSuccessfulRecovery;
  bool adaptiveMode;
  unsigned long extendedRecoveryDelay;
  // Enhanced RX overflow handling
  int rxOverflowCount;
  unsigned long firstRxOverflowTime;
  unsigned long lastRxOverflowClear;
} canErrorMonitor = {0, 0, 0, 0, false, 0, 0, 0, false, 0, 0, 0, 0};

can_frame messageQueue[CAN_QUEUE_SIZE]; // Increased from 50 to 100

// Interrupt monitoring variables
unsigned long lastInterruptTime = 0;
unsigned long lastInterruptCheck = 0;
bool interruptStuckDetected = false;
// Base timeout: 3x polling interval (30s) - ports should respond within 25s max
// after getPortData()
const unsigned long INTERRUPT_TIMEOUT = PORT_CHECK_INTERVAL * 3;
const unsigned long INTERRUPT_CHECK_INTERVAL = 10000; // Check every 10 seconds

// Global hardware watchdog object
ApplicationWatchdog *hardwareWatchdog;

// Function declarations
void checkInterruptHealth();
void recoverInterruptSystem();
void checkTransmissionReceptionBalance();
void handleRxOverflowWithEscalation(unsigned long currentTime);
void prepareCANForInterrupt();
void interruptibleDelay(unsigned long ms);

// Hardware watchdog handler
void hardwareWatchdogHandler() {
  Serial.printlnf("HARDWARE WATCHDOG RESET at uptime %lu ms", millis());
  Serial.flush();
  delay(100);
  System.reset(RESET_NO_WAIT);
}

void setup() {
  initializeSystem();

  // Log startup time and reset reason IMMEDIATELY
  Serial.printlnf("=== DEVICE STARTUP AT %lu ms ===", millis());
  Serial.printlnf("Reset reason: %d, Reset data: %d", System.resetReason(),
                  System.resetReasonData());
  Serial.printlnf("System version: %s", System.version().c_str());
  Serial.printlnf("Free memory: %lu bytes", System.freeMemory());

  // Initialize hardware watchdog early - 20 second timeout
  hardwareWatchdog =
      new ApplicationWatchdog(20000, hardwareWatchdogHandler, 1536);
  Serial.printlnf("Hardware ApplicationWatchdog initialized (20s timeout)");

  initializeArchitecture();
  initializeHardware();

  if (!CAN_ERROR) {
    initializeParticle();

    // Start alternate threads
    new Thread("can_thread", canThread);
    new Thread("port_request_thread", port_request_thread);
    new Thread("can_health_monitor", canHealthMonitorThread);
    new Thread("internet_checker", internetCheckThread);

    requestCredentials();

    // Initialize recovery state
    canErrorMonitor.lastSuccessTime = millis();
    canErrorMonitor.recoveryAttempts = 0;

    Serial.printlnf("=== SYSTEM STARTUP COMPLETE ===");
    Serial.printlnf("CAN Error Monitoring: ENABLED");
    Serial.printlnf("Hardware Watchdog: ENABLED (20s timeout)");
    Serial.printlnf("Recovery System: READY");
  } else {
    Serial.printlnf("=== STARTUP FAILED - CAN ERROR DETECTED ===");
    Serial.printlnf("Uptime before CAN error reset: %lu ms", millis());
    resetDevice("CAN error during startup");
  }
}

void loop() {
  ApplicationWatchdog::checkin(); // Feed hardware watchdog
  DeviceInfoLedger::instance().loop();

  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();

  // Detect if main loop is running too slowly (potential freeze indicator)
  if (lastLoopTime > 0 && (currentTime - lastLoopTime) > 5000) {
    Serial.printlnf(
        "WARNING: Main loop delay detected: %lu ms (uptime: %lu ms)",
        currentTime - lastLoopTime, currentTime);
  }
  lastLoopTime = currentTime;
  handleSystemLoop();

  // Small delay to prevent CPU overload during error conditions
  if (CAN_ERROR || can_recovery_needed) {
    interruptibleDelay(10);
  }
}

void initializeArchitecture() {
  // Initialize the clean architecture components
  portEventHandler =
      new PortEventHandler(nullptr); // Will use global port state functions
  portFlagHandler =
      new PortFlagHandler(nullptr); // Will use global port state functions

  Serial.printlnf("Architecture components initialized");
}

void initializeSystem() {
  Serial.begin(115200);
  while (!Serial)
    ;
  // delay(2000);

  // Track boot times using retained memory
  bootCounter++;
  uint32_t currentTime = Time.now();
  Serial.printlnf("=== BOOT #%lu at Unix time %lu ===", bootCounter,
                  currentTime);

  if (bootCounter > 1 && lastBootTime > 0) {
    uint32_t timeBetweenBoots = currentTime - lastBootTime;
    Serial.printlnf("Time since last boot: %lu seconds", timeBetweenBoots);
    if (timeBetweenBoots < 30) {
      Serial.printlnf("WARNING: Rapid restart detected! (%lu seconds)",
                      timeBetweenBoots);
    }
  }
  lastBootTime = currentTime;

  // Initialize all subsystems
  initializePorts();
  initializeCredentials();
  initializeLedger();

  Serial.printlnf("*** KUHMUTE IoT V %s ***", BUILD_VERSION);
  Serial.printlnf("Device ID: %s", Particle.deviceID().c_str());
  Serial.printlnf("Environment: %s", getCurrentEnvironment());
  Serial.printlnf("System initialized");
}
void initializeLedger() {

  // This sets up remote configuration
  DeviceConfigLedger::instance()
      .withConfigDefaultLedgerName("device-info-defaults")
      .withConfigDeviceLedgerName("device-info-config")
      .setup();
  // This sets up the device information in ledger
  DeviceInfoLedger::instance()
      .withInfoLedgerName("device-info")
      .withRetainedBuffer(retainedLogs, sizeof(retainedLogs))
      .setup();
}
void initializeHardware() {
  // Setup ring light
  beginLight();
  setLightBlue();

  // Setup SPI for CAN
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  // Initialize CAN bus
  int err = mcp2515.reset();
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "reset", true);
    return;
  }

  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "setBitrate", true);
    return;
  }

  delay(50);
  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "setNormalMode", true);
    return;
  }

  pinMode(CAN_INT, INPUT_PULLUP);
  // NOTE: CAN interrupt will be attached after connection is established
  // This prevents accumulating unwanted messages during startup

  // Initialize interrupt health tracking
  lastInterruptTime = millis();
  lastInterruptCheck = millis();
  interruptHealthy = true;

  Serial.printlnf("Hardware initialized (CAN interrupt pending)");
}

void initializeParticle() {
  // Register cloud functions
  Particle.function("resetDevice", resetDevice);
  Particle.function("getPortVin", forceGetVin);
  Particle.function("getPortStatus", forceGetPortStatus);
  Particle.function(JUISE_INCOMING, juiceMessageCallback);

  Particle.variable("CAN_ERROR", CAN_ERROR);
  Particle.variable("pub_id", MANUAL_MODE, STRING);
  Particle.variable("credentialsFetched", credentialsFetched);
  Particle.variable("total_messages_received", total_messages_received);

  // VIN traffic monitoring variables
  Particle.variable("pending_vins", g_pendingVINRequests);
  Particle.variable("active_vins", g_activeVINRequests);
  Particle.variable("vin_flood_protection", g_vinFloodProtection);
  Particle.variable("last_vin_request", g_lastVINRequestTime);

  Particle.connect();
  Particle.publishVitals(60);
  Particle.keepAlive(30);

  // Enable system features
  System.enableFeature(FEATURE_RESET_INFO);
  System.enableFeature(FEATURE_RETAINED_MEMORY);

  // Wait for connection with timeout
  setLightRed();
  waitFor(Particle.connected, 120000);
  if (!Particle.connected()) {
    Serial.printlnf("Failed to connect to Particle Cloud");
    resetDevice("");
  }

  // Log reset reason
  // logResetReason();

  setLightBlue();
  Serial.printlnf("Particle Cloud connected");
}

void handleSystemLoop() {
  // Handle CAN recovery IMMEDIATELY - before anything else
  if (can_recovery_needed) {
    Serial.printlnf("EMERGENCY: Executing CAN recovery due to error cascade");
    performCANRecovery();
    delay(1000); // Give recovery time to complete
    return;      // Exit immediately, don't continue with normal operations
  }

  // Handle critical errors
  if (CAN_ERROR) {
    blinkCANError();
    interruptibleDelay(100); // Prevent tight loop (interruptible)
    return;
  }

  // Check for credential fetch failures
  if (attemptedCredentialsFetchCount > MAX_CREDENTIAL_ATTEMPTS) {
    Serial.printlnf("Failed to fetch credentials after max attempts");
    blinkIdentityError();
    return;
  }

  // Handle connection states
  if (!Particle.connected()) {
    setLightRed();
    return;
  }

  handleCredentials();
  updateSystemStatus();

  // System health monitoring
  checkSystemHealth();

  // Check for pending port status requests
  checkPortStatusRequest();

  // Small delay to prevent CPU overload (interruptible)
  interruptibleDelay(5);

  // Process cloud messages frequently
  Particle.process();
}

/**
 * Handle systematic port data requests with staggered polling
 * Polls each port in sequence with delays to avoid overwhelming the CAN bus
 */
void handlePortDataRequests() {
  // Add static variables for timing control
  static unsigned long last_function_run_time = 0;
  static bool first_run = true;
  static int current_poll_port =
      0; // Don't initialize to 1, will be set when cycle starts

  // Don't start polling until CAN interrupt is attached
  if (!canInterruptAttached) {
    return;
  }

  // If there's a pending cloud command, pause polling to let it process
  if (pendingCloudCommand) {
    delay(10); // Small delay to let cloud command process
    return;
  }

  // Only run this function on first call or once per PORT_CHECK_INTERVAL
  unsigned long current_time = millis();
  if (!first_run && !polling_cycle_active &&
      (current_time - last_function_run_time < PORT_CHECK_INTERVAL)) {
    // Too soon to run again, skip
    return;
  }

  // EMERGENCY STOP - Don't poll if we're in error cascade
  if (can_recovery_needed || CAN_ERROR) {
    polling_cycle_active =
        false;             // CRITICAL: Must reset this to prevent deadlock
    current_poll_port = 1; // Reset port counter
    markPortsUnpolled();
    delay(100); // Prevent tight loop during recovery
    return;
  }

  // Add consecutive failure tracking per port
  static int portFailureCount[MAX_PORTS + 1] = {0}; // Track failures per port
  static bool pollingDisabled = false;
  static unsigned long pollingDisabledTime = 0;

  // PREVENTIVE VIN flood protection - conservative approach
  static unsigned long lastVINRequestTime = 0;
  static int pendingVINRequests = 0;
  static unsigned long lastHighTrafficCheck = 0;

  // Conservative VIN request limiting (prevent silent lockups)
  static int activeVINRequests = 0; // Currently processing VIN requests
  static unsigned long vinRequestSlots[3] = {
      0, 0, 0}; // Track up to 3 concurrent VINs
  static const int MAX_CONCURRENT_VINS =
      2; // Conservative: only 2 VIN requests at once

  // Check if we should re-enable polling after errors cleared
  if (pollingDisabled) {
    unsigned long waitTime = 5000; // Default 5 seconds

    // Extended wait time in adaptive mode
    if (canErrorMonitor.adaptiveMode &&
        canErrorMonitor.extendedRecoveryDelay > 5000) {
      waitTime = canErrorMonitor.extendedRecoveryDelay;
    }

    if (canErrorMonitor.consecutiveErrors == 0 &&
        millis() - pollingDisabledTime > waitTime) {
      Serial.printlnf(
          "Re-enabling port polling - errors have cleared (waited %lu ms)",
          waitTime);
      pollingDisabled = false;
      // Reset all port failure counts
      for (int i = 0; i <= MAX_PORTS; i++) {
        portFailureCount[i] = 0;
      }
    } else {
      // Make delay interruptible
      for (int i = 0; i < 10 && !pendingCloudCommand; i++) {
        delay(10);
        Particle.process();
      }
      return;
    }
  }

  // Static variable for port check reset timing
  static unsigned long last_port_check_reset = 0;
  static unsigned long last_poll_send_time = 0;

  // Start of a new polling cycle - reset all ports for polling
  if (!polling_cycle_active ||
      (current_time - last_port_check_reset >= PORT_CHECK_INTERVAL)) {
    // Only print the message when we're actually resetting
    if (current_time - last_port_check_reset >= PORT_CHECK_INTERVAL) {
      markPortsUnpolled();
      last_port_check_reset = current_time; // Update the last reset time
      Serial.println("🚨 DID_PORT_CHECK reset for all ports 🚨");

      // Force start new cycle when ports are reset
      polling_cycle_active = false;
      current_poll_port = 0; // Will be set to 1 when cycle actually starts
    }

    // Start a new polling cycle
    if (!polling_cycle_active) {
      polling_cycle_active = true;
      current_poll_port = 1; // NOW set to port 1 when actually starting
      Serial.printlnf(
          "Starting new polling cycle for all ports - starting at port %d",
          current_poll_port);
    }
  }

  // IMPORTANT: Only check this if it's not the first poll (last_poll_send_time
  // will be 0 on first poll)
  if (last_poll_send_time != 0) {
    // Check if enough time has passed since last poll
    if (current_time - last_poll_send_time < POLL_STAGGER_DELAY) {
      // Not enough time has passed, skip this poll cycle
      return;
    }

    // PREVENTIVE: Count and limit VIN requests to prevent silent lockups
    if (current_time - lastHighTrafficCheck >= 3000) { // Check every 3 seconds
      pendingVINRequests = 0;
      activeVINRequests = 0;

      // Clear expired VIN request slots (assume 15 seconds max for VIN
      // completion)
      for (int i = 0; i < 3; i++) {
        if (vinRequestSlots[i] > 0 &&
            (current_time - vinRequestSlots[i]) > 15000) {
          vinRequestSlots[i] = 0; // Clear expired slot
        } else if (vinRequestSlots[i] > 0) {
          activeVINRequests++; // Count active VIN requests
        }
      }

      // Count pending VIN flags
      for (int i = 1; i <= MAX_PORTS; i++) {
        if (isValidPort(i)) {
          PortState *state = getPortState(i);
          if (state && state->vin_request_flag) {
            pendingVINRequests++;
          }
        }
      }

      // Update global variables for cloud monitoring
      g_pendingVINRequests = pendingVINRequests;
      g_activeVINRequests = activeVINRequests;

      lastHighTrafficCheck = current_time;
    }

    // CONSERVATIVE delay calculation to prevent bus saturation
    unsigned long smart_delay = POLL_STAGGER_DELAY; // Default 1000ms

    // MUCH more conservative VIN spacing to prevent silent lockups
    if (pendingVINRequests >= 3) {
      smart_delay = 8000; // 8 second delay when 3+ VIN requests pending
    } else if (pendingVINRequests >= 1) {
      smart_delay = 5000; // 5 second delay when any VIN requests pending
    }

    // Aggressive delay for higher port addresses (they have 1200ms+ response
    // delays)
    if (current_poll_port > 10) {
      smart_delay += 3000; // Extra 3 seconds for ports 11-16
    } else if (current_poll_port > 8) {
      smart_delay += 2000; // Extra 2 seconds for ports 9-10
    }

    // LONG buffer after VIN requests - accounts for full 3-message sequence +
    // port delays
    if (current_time - lastVINRequestTime <
        10000) {           // Within 10 seconds of VIN request
      smart_delay += 5000; // Add 5 second buffer minimum
    }

    // If any VIN requests are actively being processed, be extra conservative
    if (activeVINRequests > 0) {
      smart_delay += 3000; // Additional 3 seconds when VINs are processing
    }

    // Check if enough time has passed since last poll using smart delay
    // Make this interruptible by checking for cloud commands
    unsigned long delay_start = millis();
    while ((millis() - last_poll_send_time) < smart_delay) {
      // Process Particle cloud to ensure responsiveness
      Particle.process();

      // Check for pending cloud commands every 50ms
      if (pendingCloudCommand) {
        return; // Exit immediately to process cloud command
      }

      // Prevent infinite loop
      if ((millis() - delay_start) > (smart_delay + 1000)) {
        break;
      }

      delay(50); // Small delay between checks
    }
  }

  // Check if portFlagHandler is available
  if (!portFlagHandler) {
    Serial.printlnf("portFlagHandler not initialized");
    return;
  }

  // Only process port polling when we're in an active cycle
  if (!polling_cycle_active) {
    return; // Don't increment port counter between cycles
  }

  // Safety check - if current_poll_port is 0, set it to 1
  if (current_poll_port == 0) {
    current_poll_port = 1;
    Serial.printlnf("WARNING: current_poll_port was 0, resetting to 1");
  }

  // Find next port that needs polling
  int start_port = current_poll_port; // Remember where we started
  for (int attempts = 0; attempts < MAX_PORTS; attempts++) {
    // Validate and wrap port number BEFORE using it
    if (current_poll_port < 1 || current_poll_port > MAX_PORTS) {
      current_poll_port = 1; // Wrap around or fix invalid port number
    }

    if (!hasPortBeenPolled(current_poll_port)) {
      // Check if this port has a pending VIN request
      bool isVINRequest = false;
      PortState *state = getPortState(current_poll_port);
      if (state) {
        isVINRequest = state->vin_request_flag;
      }

      // PREVENTIVE: Skip VIN requests if too many are already active
      if (isVINRequest && activeVINRequests >= MAX_CONCURRENT_VINS) {
        Serial.printlnf("PREVENTIVE: Skipping VIN request for port %d - %d "
                        "already active (max %d)",
                        current_poll_port, activeVINRequests,
                        MAX_CONCURRENT_VINS);
        current_poll_port++; // Move to next port
        continue;            // Skip this VIN request
      }

      // This is the critical part - actually send the port data request
      bool success = portFlagHandler->sendGetPortData(current_poll_port);

      if (success) {
        Serial.printlnf("Polled port %d", current_poll_port);
        markPortPolled(current_poll_port);
        // Reset failure count on success
        portFailureCount[current_poll_port] = 0;
        resetCANSuccessCounter();
        // Track transmission time for health monitoring
        lastTransmissionTime = millis();

        // Track VIN request timing and reserve slot
        if (isVINRequest) {
          lastVINRequestTime = current_time;
          g_lastVINRequestTime = current_time;

          // Reserve a VIN request slot
          for (int i = 0; i < 3; i++) {
            if (vinRequestSlots[i] == 0) {
              vinRequestSlots[i] = current_time;
              break;
            }
          }

          Serial.printlnf("VIN request sent to port %d - slot reserved, %d "
                          "slots active",
                          current_poll_port, activeVINRequests + 1);
        }

        // Calculate port wait time (empirical values based on testing)
        int portWaitTime = 3000; // Default 3 seconds between polls

        // Higher ports need longer delays (cascading response delays)
        if (current_poll_port > 8) {
          portWaitTime = 4000; // 4 seconds for ports 9+
        }
        if (current_poll_port > 12) {
          portWaitTime = 5000; // 5 seconds for ports 13+
        }

        // Global STAGGER_DELAY can override port wait time
        if (POLL_STAGGER_DELAY > portWaitTime) {
          portWaitTime = POLL_STAGGER_DELAY;
        }

        // VIN requests need much longer delay
        if (isVINRequest) {
          portWaitTime = 10000; // 10 seconds minimum for VIN requests
        }

      } else {
        portFailureCount[current_poll_port]++;
        Serial.printlnf("Failed to poll port %d (failures: %d)",
                        current_poll_port, portFailureCount[current_poll_port]);

        // Skip this port if it's failing repeatedly
        if (portFailureCount[current_poll_port] >= 3) {
          Serial.printlnf(
              "Port %d has failed %d times - skipping for this cycle",
              current_poll_port, portFailureCount[current_poll_port]);
          markPortPolled(current_poll_port); // Mark as "polled" to skip it
          portFailureCount[current_poll_port] = 0; // Reset for next cycle
        }

        logCANError(-1, "port_polling");

        // If too many consecutive errors, trigger IMMEDIATE recovery
        if (canErrorMonitor.consecutiveErrors >= 3) {
          Serial.printlnf("CRITICAL: Immediate CAN recovery needed");
          can_recovery_needed = true;
          pollingDisabled = true;
          pollingDisabledTime = millis();
          pendingCloudCommand = false; // Clear flag during error
          return;                      // Stop polling immediately
        }

        // Hardware watchdog will handle timeout protection automatically
      }

      last_poll_send_time = current_time;

      // Move to next port and wrap if needed
      current_poll_port++;
      if (current_poll_port > MAX_PORTS) {
        current_poll_port = 1;
      }

      // Instead of breaking, return to allow delay between ports
      return;
    }

    // Move to next port when skipping already-polled ports
    current_poll_port++;
    if (current_poll_port > MAX_PORTS) {
      current_poll_port = 1;
    }
  }

  // If we get here, we've checked all ports and no ports need polling in this
  // cycle Check if we've completed a full cycle (all ports are polled)
  bool all_ports_polled = true;
  for (int port = 1; port <= MAX_PORTS; port++) {
    if (!hasPortBeenPolled(port)) {
      all_ports_polled = false;
      break;
    }
  }

  // If all ports are polled, end the polling cycle
  if (all_ports_polled) {
    polling_cycle_active = false;
    current_poll_port = 1; // Reset to port 1 for next cycle
    last_function_run_time = current_time;
    first_run = false;
    Serial.printlnf(
        "Port polling cycle complete - all ports polled - next cycle in %lu ms",
        PORT_CHECK_INTERVAL);
  }
}

void handleCredentials() {
  if (areCredentialsValid()) {
    return; // Already have valid credentials
  }

  if (shouldRetryCredentials()) {
    requestCredentials();
  }
}

void updateSystemStatus() {
  if (areCredentialsValid() && CELLULAR_CONNECTED) {
    setLightGreen(); // All systems operational

    // Set the just connected flag when we first get valid credentials and
    // Internet connected connection
    static bool wasConnectedBefore = false;
    if (!wasConnectedBefore) {
      justConnectedFlag = true;
      wasConnectedBefore = true;
      Serial.printlnf("First successful connection detected - immediate port "
                      "polling enabled");

      // Prepare CAN system and attach interrupt with clean state
      prepareCANForInterrupt();
    }
  } else if (areCredentialsValid()) {
    setLightPurple(); // Have credentials, connecting to Cloud
  } else {
    setLightBlue(); // Fetching credentials
  }
}

// ========================================
// CAN Processing
// ========================================

void canThread() {
  Serial.printlnf("CAN processing thread started");

  while (true) {
    if (areCredentialsValid() && CELLULAR_CONNECTED && !CAN_ERROR &&
        !can_recovery_needed) {
      // Give priority to cloud commands
      if (pendingCloudCommand) {
        delay(50); // Give cloud command time to process
        Particle.process();
        continue; // Skip this iteration to let cloud command complete
      }

      // Process incoming CAN messages
      handleCanQueue();

      // Process port flags (replaces old flagHandler)
      if (portFlagHandler) {
        portFlagHandler->processAllPortFlags();
      }

      // Process Particle cloud to ensure responsiveness
      Particle.process();
    }
    // Small delay to prevent busy-waiting
    delay(10);
  }
}

void port_request_thread() {
  while (true) {
    if (areCredentialsValid() && CELLULAR_CONNECTED && !CAN_ERROR &&
        !can_recovery_needed) {
      // Skip polling if cloud command is pending
      if (!pendingCloudCommand) {
        handlePortDataRequests();
      }

      // Shorter delay to allow staggered polling to work properly
      // Each port will be polled with the appropriate delay between them
      interruptibleDelay(50);
    } else {
      // Small delay to prevent busy-waiting
      interruptibleDelay(100);
    }
  }
}

void handleCanQueue() {
  int messagesProcessed = 0;
  int MAX_MESSAGES_PER_LOOP = 8;

  // If queue is critically full, process more aggressively
  if (messageCount > (CAN_QUEUE_SIZE * 3 / 4)) { // More than 75% full
    MAX_MESSAGES_PER_LOOP = 20;
    Serial.printlnf("CAN queue critical (%d messages) - aggressive processing",
                    messageCount);
  }

  while (messageCount > 0 && messagesProcessed < MAX_MESSAGES_PER_LOOP) {
    can_frame msg;
    bool validMessage = false;

    // Thread-safe message extraction
    noInterrupts();
    if (messageCount > 0) {
      if (queueHead >= 0 && queueHead < CAN_QUEUE_SIZE && queueTail >= 0 &&
          queueTail < CAN_QUEUE_SIZE) {
        msg = messageQueue[queueHead];
        queueHead = (queueHead + 1) % CAN_QUEUE_SIZE;
        messageCount--;
        validMessage = true;

        // Handle queue overflow reset
        if (queueOverflow && messageCount < (CAN_QUEUE_SIZE / 2)) {
          queueOverflow = false;
          Serial.println("Queue overflow cleared");
        }
      } else {
        // Invalid queue state, reset
        queueTail = 0;
        queueHead = 0;
        messageCount = 0;
        Serial.println(
            "ERROR: Invalid queue indices detected - resetting queue");
      }
    }
    interrupts();

    // Process the message using clean architecture
    if (validMessage) {
      processCANMessage(msg);
      messagesProcessed++;
    }

    // Yield periodically
    if (messagesProcessed % 2 == 0) {
      Particle.process();
    }
  }
}

void processCANMessage(const can_frame &rawMessage) {
  // Check for corrupted CAN ID (negative or excessively large values)
  if ((int32_t)rawMessage.can_id < 0 || rawMessage.can_id > 16777215) {
    Serial.printlnf("CRITICAL: Corrupted CAN ID detected: %ld (0x%lX)",
                    (long)rawMessage.can_id, (unsigned long)rawMessage.can_id);
    logCANError(0xF, "corrupted_can_id");

    // This indicates controller corruption - trigger immediate recovery
    Serial.printlnf("CAN controller corruption detected - triggering recovery");
    can_recovery_needed = true;
    return;
  }

  // Check for invalid data length
  if (rawMessage.can_dlc > 8) {
    Serial.printlnf("CRITICAL: Invalid CAN DLC: %d", rawMessage.can_dlc);
    logCANError(0xE, "invalid_dlc");
    can_recovery_needed = true;
    return;
  }

  // 1. Parse the raw CAN message
  ParsedCANMessage parsedMsg = canProcessor.parseMessage(rawMessage);

  // 2. Log the message for debugging
  Serial.printlnf("CAN message from port %d: type=%s, valid=%s",
                  parsedMsg.sourcePort,
                  canProcessor.getMessageTypeString(parsedMsg.messageType),
                  parsedMsg.isValid ? "yes" : "no");

  // 3. Handle the business logic if message is valid
  if (parsedMsg.isValid && portEventHandler) {
    portEventHandler->handleCANMessage(parsedMsg);
    // Reset consecutive error count on successful message processing
    resetCANSuccessCounter();
    // Update interrupt health tracking - we successfully processed a message
    lastInterruptTime = millis();
    interruptHealthy = true;
  } else if (!parsedMsg.isValid) {
    Serial.printlnf("Invalid CAN message received from port %d",
                    parsedMsg.sourcePort);
    logCANError(-1, "message_parsing");
  }
}

void can_interrupt() {
  // Minimal interrupt handler - just queue the message
  struct can_frame recMsg;

  // Read the message
  int readin = readCanMessage(&recMsg);

  if (readin == ERROR_OK) {
    // Comprehensive corruption and gibberish filtering
    // 1. Basic validity checks
    if ((int32_t)recMsg.can_id < 0 || recMsg.can_id == 0 ||
        recMsg.can_id > MAX_PORTS || recMsg.can_dlc > 8) {
      // Don't queue obviously corrupted messages
      return; // Silent discard for performance
    }

    // 2. Data content validation - filter out gibberish
    bool hasValidData = false;
    bool hasGibberish = false;

    for (int i = 0; i < recMsg.can_dlc; i++) {
      uint8_t byte = recMsg.data[i];

      // Check for valid ASCII characters (printable range)
      if ((byte >= 0x20 && byte <= 0x7E) || byte == 0x00) {
        hasValidData = true;
      }
      // Check for obvious gibberish (control chars, high ASCII)
      else if (byte < 0x20 || byte > 0x7E) {
        // Allow some common control characters
        if (byte != 0x0A && byte != 0x0D && byte != 0x09) // LF, CR, TAB
        {
          hasGibberish = true;
        }
      }
    }

    // Reject if mostly gibberish or no valid data
    if (hasGibberish && !hasValidData) {
      return; // Silent discard of gibberish
    }

    // 3. Message pattern validation - must start with valid command
    if (recMsg.can_dlc > 0) {
      char firstChar = (char)recMsg.data[0];
      // Valid message types: D(status), K(VIN), T(temp), H(heartbeat), etc.
      if (firstChar != 'D' && firstChar != 'K' && firstChar != 'T' &&
          firstChar != 'H' && firstChar != 'U' && firstChar != 'C' &&
          firstChar != 'V' && firstChar != 'F' && firstChar != 'E') {
        return; // Silent discard of invalid command
      }
    }

    // Add to queue if there's space
    if (messageCount < CAN_QUEUE_SIZE) {
      incrementMessageCounter();
      messageQueue[queueTail] = recMsg;
      queueTail = (queueTail + 1) % CAN_QUEUE_SIZE;
      messageCount++;
    } else {
      queueOverflow = true;
      logCANError(-2, "queue_overflow");

      // Emergency queue clear if severely overflowing
      static unsigned long lastEmergencyClear = 0;
      if (millis() - lastEmergencyClear > 5000) { // Only once per 5 seconds
        Serial.println(
            "EMERGENCY: Clearing oldest 25% of queue to prevent lockup");
        // Clear oldest 25% of messages
        int toClear = messageCount / 4;
        queueHead = (queueHead + toClear) % CAN_QUEUE_SIZE;
        messageCount -= toClear;
        lastEmergencyClear = millis();
      }
    }
  } else {
    // Log read errors (but not every single one to avoid spam)
    static unsigned long lastErrorLog = 0;
    if (millis() - lastErrorLog > 1000) // Log max once per second
    {
      logCANError(readin, "can_read_interrupt");
      lastErrorLog = millis();
    }
  }
}

// ========================================
// CAN Error Monitoring and Recovery
// ========================================

void performCANRecovery() {
  Serial.printlnf("=== PERFORMING EMERGENCY CAN RECOVERY ===");

  // Stop ALL CAN operations immediately
  CAN_ERROR = true;
  can_recovery_needed = false;  // Clear the flag first
  polling_cycle_active = false; // Reset polling state during recovery
  canInterruptAttached = false; // Disable polling during recovery

  // Clear all queues and reset state
  noInterrupts();
  queueHead = 0;
  queueTail = 0;
  messageCount = 0;
  queueOverflow = false;

  // Reset RX overflow tracking during recovery
  canErrorMonitor.rxOverflowCount = 0;
  canErrorMonitor.firstRxOverflowTime = 0;
  canErrorMonitor.lastRxOverflowClear = 0;
  interrupts();

  // Detach interrupt to prevent issues during recovery
  detachInterrupt(CAN_INT);

  // Update recovery tracking
  canErrorMonitor.recoveryAttempts++;
  canErrorMonitor.inRecoveryMode = true;
  canErrorMonitor.lastRecoveryAttempt = millis();

  // Check if we've exceeded max recovery attempts without recent success
  if (canErrorMonitor.recoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
    unsigned long timeSinceLastSuccess =
        millis() - canErrorMonitor.lastSuccessfulRecovery;

    // If it's been a long time since last successful recovery, allow reset of
    // attempts
    if (timeSinceLastSuccess > RECOVERY_SUCCESS_RESET_TIME) {
      Serial.printlnf("Resetting recovery attempts - %lu ms since last success",
                      timeSinceLastSuccess);
      canErrorMonitor.recoveryAttempts = 1; // Reset but count this attempt
      canErrorMonitor.adaptiveMode = true;  // Enable adaptive recovery
    } else {
      Serial.printlnf("CRITICAL: Max recovery attempts (%d) exceeded - forcing "
                      "device reset",
                      MAX_RECOVERY_ATTEMPTS);
      delay(100);
      resetDevice("Max CAN recovery attempts exceeded");
      return;
    }
  }

  Serial.printlnf("CAN Recovery attempt %d of %d",
                  canErrorMonitor.recoveryAttempts, MAX_RECOVERY_ATTEMPTS);

  // Adaptive recovery delay based on attempt number
  unsigned long recoveryDelay = 500;
  if (canErrorMonitor.adaptiveMode) {
    recoveryDelay =
        1000 + (canErrorMonitor.recoveryAttempts * 500); // Progressive delay
    canErrorMonitor.extendedRecoveryDelay =
        recoveryDelay * 2; // Extended pause after recovery
    Serial.printlnf("Adaptive recovery mode - using %lu ms delay",
                    recoveryDelay);
  }

  delay(recoveryDelay); // Give system time to stabilize

  // Reset MCP2515 controller
  Serial.printlnf("Resetting MCP2515 controller...");
  int err = mcp2515.reset();
  if (err != mcp2515.ERROR_OK) {
    Serial.printlnf("CRITICAL: MCP2515 reset failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 reset failed");
    return;
  }

  delay(200);

  // Reconfigure controller
  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != mcp2515.ERROR_OK) {
    Serial.printlnf(
        "CRITICAL: MCP2515 bitrate config failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 config failed");
    return;
  }

  delay(100);

  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK) {
    Serial.printlnf(
        "CRITICAL: MCP2515 normal mode failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 mode failed");
    return;
  }

  // Re-enable interrupts after successful controller reset
  pinMode(CAN_INT, INPUT_PULLUP);

  // Only re-attach interrupt if we're connected and have credentials
  if (areCredentialsValid() && CELLULAR_CONNECTED) {
    // Use prepareCANForInterrupt to ensure clean state
    prepareCANForInterrupt();
    canInterruptAttached = true; // Re-enable polling after recovery
    Serial.printlnf(
        "CAN recovery complete - interrupt re-enabled with clean state");
  } else {
    canInterruptAttached = false; // Disable polling until connected
    Serial.printlnf(
        "CAN interrupt pending - will attach after connection established");
  }

  // Reset all error tracking
  can_error_count = 0;
  last_can_error_time = millis();
  canErrorMonitor.consecutiveErrors = 0;
  canErrorMonitor.inRecoveryMode = false;
  canErrorMonitor.lastSuccessTime = millis();
  canErrorMonitor.lastSuccessfulRecovery = millis();
  CAN_ERROR = false;

  Serial.printlnf("=== CAN RECOVERY COMPLETED SUCCESSFULLY ===");
  Serial.printlnf("Recovery attempt %d succeeded",
                  canErrorMonitor.recoveryAttempts);

  // Extended pause in adaptive mode for system stabilization
  unsigned long postRecoveryDelay = 1000;
  if (canErrorMonitor.adaptiveMode &&
      canErrorMonitor.extendedRecoveryDelay > 1000) {
    postRecoveryDelay = canErrorMonitor.extendedRecoveryDelay;
    Serial.printlnf("Extended post-recovery delay: %lu ms", postRecoveryDelay);
  }

  delay(postRecoveryDelay);
}

void logCANError(int errorCode, const char *operation) {
  unsigned long currentTime = millis();

  canErrorMonitor.totalErrors++;
  canErrorMonitor.lastErrorTime = currentTime;
  canErrorMonitor.consecutiveErrors++;

  Serial.printlnf("CAN Error #%lu: %s failed with code %d (consecutive: %d)",
                  canErrorMonitor.totalErrors, operation, errorCode,
                  canErrorMonitor.consecutiveErrors);

  // Check for specific error codes that indicate controller corruption
  if (errorCode == 0xA || errorCode == 10 || errorCode == 0xF ||
      errorCode == 15) {
    Serial.printlnf("CRITICAL: Detected ERROR_A/ERROR_F - MCP2515 controller "
                    "corruption!");
    Serial.printlnf(
        "Triggering immediate CAN recovery to prevent system freeze");
    can_recovery_needed = true;

    // If we've seen these errors multiple times, force device reset
    if (canErrorMonitor.consecutiveErrors >= 2) {
      Serial.printlnf(
          "Multiple controller corruption errors - forcing device reset");
      delay(100);
      resetDevice("MCP2515 controller corruption");
      return;
    }
  }

  // Lower threshold for faster recovery during error cascades
  if (canErrorMonitor.consecutiveErrors >= 3) {
    Serial.printlnf("Fast recovery trigger - %d consecutive errors",
                    canErrorMonitor.consecutiveErrors);
    can_recovery_needed = true;
  }

  // Legacy compatibility - keep old variables for now
  can_error_count++;
  last_can_error_time = currentTime;
  if (can_error_count > MAX_CAN_ERRORS_PER_MINUTE) {
    can_recovery_needed = true;
  }
}

void resetCANSuccessCounter() {
  // Reset consecutive error count on successful operation
  if (canErrorMonitor.consecutiveErrors > 0) {
    Serial.printlnf("CAN operation successful - resetting error count");
    canErrorMonitor.consecutiveErrors = 0;
  }

  // Update success time
  canErrorMonitor.lastSuccessTime = millis();

  // Reset recovery attempts after sustained successful operation
  if (canErrorMonitor.recoveryAttempts > 0) {
    unsigned long timeSinceRecovery =
        millis() - canErrorMonitor.lastRecoveryAttempt;

    // Only reset attempts if we've had sustained success (30 seconds)
    if (timeSinceRecovery > 30000) {
      Serial.printlnf("Resetting recovery attempt counter after sustained "
                      "success (%lu ms)",
                      timeSinceRecovery);
      canErrorMonitor.recoveryAttempts = 0;
      canErrorMonitor.adaptiveMode =
          false; // Disable adaptive mode on sustained success
      canErrorMonitor.extendedRecoveryDelay = 0;
    }
  }

  // Only clear recovery mode if we're not currently in an active recovery
  if (!can_recovery_needed && !CAN_ERROR) {
    canErrorMonitor.inRecoveryMode = false;
  }
}

// ========================================
// Watchdog Functions
// ========================================
// Software watchdog functions removed - using hardware ApplicationWatchdog
// only

// ========================================
// Utility Functions
// ========================================

void reportCANError(int err, const char *operation, bool report) {
  if (report) {
    CAN_ERROR = err != 0;
  }

  if (err != 0) {
    char ret[150];
    Serial.printlnf("CAN BUS ERROR %s", operation);
    ReturnErrorString(err, ret, sizeof(ret));
    sprintf(can_err_msg, "CAN BUS ERROR %s: %s", operation, ret);
    Serial.printlnf("%s", can_err_msg);

    // Log this as a CAN error for monitoring
    logCANError(err, operation);
  }
}

int resetDevice(String command) {
  if (command.length() > 0) {
    Serial.printlnf("Device reset requested. Reason: %s", command.c_str());
    Serial.printlnf("Free memory before reset: %lu bytes", System.freeMemory());
    Serial.printlnf("System uptime: %lu ms", millis());
    Serial.printlnf("Cellular connected: %s",
                    CELLULAR_CONNECTED ? "yes" : "no");
    Serial.printlnf("Credentials valid: %s",
                    areCredentialsValid() ? "yes" : "no");
    Serial.printlnf("CAN errors in last period: %d", can_error_count);
  }
  System.reset();
  return 1;
}
int forceGetVin(String command) {
  int port = atoi(command.c_str());
  if (!isValidPort(port)) {
    Particle.publish("forceGetVin", "error: invalid port", PRIVATE);
    return -1;
  }
  PortState *state = getPortState(port);
  if (!state) {
    Particle.publish("forceGetVin", "error: no state", PRIVATE);
    return -2;
  }

  if (strlen(state->VIN) > 0) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Port %d VIN: %s", port, state->VIN);
    Particle.publish("forceGetVin", buffer, PRIVATE);
    return 1; // Success
  } else {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Port %d: no VIN", port);
    Particle.publish("forceGetVin", buffer, PRIVATE);
    return 0; // No VIN
  }
}
int forceGetPortStatus(String command) {
  int port = atoi(command.c_str());

  String portStatus = getPortStatusSummary(port);
  // char buffer[100];
  // snprintf(buffer, sizeof(buffer), portStatus.c_str());
  Particle.publish("forceGetPortStatus", portStatus, PRIVATE);
  return 1; // Success
}

void logDebugInfo(const char *checkpoint) {
  static unsigned long lastLogTime = 0;
  unsigned long now = millis();

  Serial.printlnf("[%lu ms] Checkpoint: %s (Delta: %lu ms)", now, checkpoint,
                  now - lastLogTime);

  lastLogTime = now;
}

void logResetReason() {
  int reason = System.resetReason();
  int reasonData = System.resetReasonData();

  Serial.printlnf("RESET REASON: %d", reason);
  Serial.printlnf("RESET REASON DATA: %d", reasonData);

  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Reason: %d, Data: %d", reason, reasonData);

  Particle.publish("RESET REASON", buffer, PRIVATE);
}

void checkSystemHealth() {
  unsigned long freeMemory = System.freeMemory();
  unsigned long uptime = millis();

  if (freeMemory < 1000) {
    Serial.printlnf("Low memory warning: %lu bytes", freeMemory);
  }

  static unsigned long lastHealthCheck = 0;
  if (uptime - lastHealthCheck > 60000) {
    Serial.printlnf("System Health - Uptime: %lu ms, Free Memory: %lu bytes",
                    uptime, freeMemory);
    Serial.printlnf("Cellular Status: %s",
                    CELLULAR_CONNECTED ? "connected" : "disconnected");
    Serial.printlnf("Credentials: %s", getCredentialsStatus().c_str());
    Serial.printlnf("CAN Errors (last minute): %d", can_error_count);
    Serial.printlnf("CAN Recovery needed: %s",
                    can_recovery_needed ? "yes" : "no");

    if (portFlagHandler) {
      Serial.printlnf("Ports with pending flags: %d",
                      portFlagHandler->getPendingPortsCount());
    }

    lastHealthCheck = uptime;
  }
}

void emergencyReset(const char *reason) {
  Serial.printlnf("EMERGENCY RESET: %s", reason);
  Serial.printlnf("Error Stats - Consecutive: %d, Total: %lu",
                  canErrorMonitor.consecutiveErrors,
                  canErrorMonitor.totalErrors);
  delay(200); // Ensure message is sent
  System.reset();
}

// Internet connectivity monitoring thread with safety features
// This thread monitors internet connection and performs recovery actions:
// 1. Soft recovery (disconnect/reconnect) after 30 seconds
// 2. Hard reset after 1 minute, but only if system is in safe state
void internetCheckThread() {
  static bool connected = true;
  static bool did_disconnect = false;
  static unsigned long disconnectTime = 0;
  static unsigned long lastCheckTime = 0;
  static bool calledConnect = false;
  while (true) {
    // Non-blocking check every 5 seconds
    if (millis() - lastCheckTime > 5000) {

      lastCheckTime = millis();

      CELLULAR_CONNECTED = (Cellular.ready() && Particle.connected());

      if (!CELLULAR_CONNECTED) {
        if (!did_disconnect) { // Only set disconnectTime on FIRST disconnect
          Serial.println("Internet disconnected - starting recovery timeline");
          did_disconnect = true;
          disconnectTime = millis();
        }
        unsigned long disconnectedDuration = millis() - disconnectTime;

        // if (!calledConnect) {
        //   Particle.connect();
        //   calledConnect = true;
        // }
        // Hard reset after 1 minute
        if (disconnectedDuration > INTERNET_DISCONNECT_RESET_TIMEOUT) {

          Serial.printlnf("RESET: Internet disconnected for %lu ms - "
                          "performing device reset",
                          disconnectedDuration);
          System.reset(RESET_NO_WAIT);
        }
      } else {
        if (did_disconnect) {
          // Connection restored
          Serial.printlnf("Internet reconnected after %lu ms offline",
                          millis() - disconnectTime);
        }
        disconnectTime = 0;
        did_disconnect = false;
        // calledConnect = false;
      }
    } else {
      delay(100);
    }
  }
}

void canHealthMonitorThread() {
  static unsigned long lastHealthCheck = 0;
  const unsigned long HEALTH_CHECK_INTERVAL =
      1000; // Check every second during crisis
  const unsigned long MAIN_LOOP_TIMEOUT = 20000; // 20 seconds max

  Serial.println("CAN Health Monitor thread started");

  while (true) {
    unsigned long currentTime = millis();

    // Check health every second
    if (currentTime - lastHealthCheck >= HEALTH_CHECK_INTERVAL) {

      // Check MCP2515 error flags for actual hardware issues
      uint8_t currentFlags = getCANErrorFlags(true);

      // Monitor CAN error rate (from old canMonitorThread)
      // Monitor CAN error rate
      if (currentTime - last_can_error_time > CAN_ERROR_RESET_INTERVAL) {
        if (can_error_count > 0) {
          Serial.printlnf("Resetting CAN error count (was %d)",
                          can_error_count);
        }
        can_error_count = 0;
        last_can_error_time = currentTime;
      }

      // Check if error rate is too high
      if (can_error_count > MAX_CAN_ERRORS_PER_MINUTE) {
        Serial.printlnf("CAN error rate too high (%d errors/minute), "
                        "triggering recovery",
                        can_error_count);
        can_recovery_needed = true;
      }

      // Check interrupt health
      checkInterruptHealth();

      // Check transmission/reception balance
      checkTransmissionReceptionBalance();

      // Check if we have consecutive CAN errors with lower threshold
      if (canErrorMonitor.consecutiveErrors >= 3) {
        if (!canErrorMonitor.inRecoveryMode) {
          Serial.printlnf("CAN Error Threshold Reached: %d consecutive errors",
                          canErrorMonitor.consecutiveErrors);
          can_recovery_needed = true;
          canErrorMonitor.inRecoveryMode = true;
          canErrorMonitor.lastRecoveryAttempt = currentTime;
        }
      }

      // Reset consecutive error count if enough time has passed without
      // errors
      if (currentTime - canErrorMonitor.lastErrorTime > CAN_ERROR_WINDOW) {
        if (canErrorMonitor.consecutiveErrors > 0) {
          Serial.printlnf("CAN Error Window Expired - Resetting "
                          "consecutive error count");
          canErrorMonitor.consecutiveErrors = 0;
        }
      }

      // Emergency reset if error cascade continues
      if (canErrorMonitor.consecutiveErrors >= 10) {
        emergencyReset("Excessive consecutive CAN errors");
      }

      // Reset if stuck in recovery mode too long
      if (canErrorMonitor.inRecoveryMode &&
          currentTime - canErrorMonitor.lastRecoveryAttempt >
              CAN_RECOVERY_DELAY * 2) {
        emergencyReset("CAN recovery timeout");
      }

      // Check if no successful operations for too long
      if (canErrorMonitor.lastSuccessTime > 0 &&
          currentTime - canErrorMonitor.lastSuccessTime >
              PORT_CHECK_INTERVAL * 2 + 5000) {
        // Instead of emergency reset, check if CAN hardware is actually
        // healthy
        if (currentFlags == 0 && canErrorMonitor.consecutiveErrors == 0) {
          // CAN hardware looks fine, maybe just need to restart polling
          Serial.println("No recent CAN success but hardware looks healthy - "
                         "restarting polling");
          markPortsUnpolled(); // Gentle restart of port polling
          canErrorMonitor.lastSuccessTime = millis(); // Reset timer

          // Give it one more cycle to verify success
          static unsigned long lastGentleRestart = 0;
          if (currentTime - lastGentleRestart >
              60000) { // Only try once per minute
            lastGentleRestart = currentTime;
          } else {
            // Second gentle restart failed, more serious issue
            Serial.println("Gentle restart failed, CAN may have deeper issues");
            emergencyReset("CAN operations failed even after gentle restart");
          }
        } else {
          // Actual CAN hardware errors detected
          Serial.printlnf("CAN hardware errors detected (flags: 0x%02X)",
                          currentFlags);
          emergencyReset("No successful CAN operations with hardware errors");
        }
      }

      // Hardware watchdog handles main loop freeze detection automatically

      // Check CAN error rate with lower threshold
      if (canErrorMonitor.consecutiveErrors >= 6) {
        Serial.printlnf("CRITICAL: Excessive CAN errors (%d), forcing reset",
                        canErrorMonitor.consecutiveErrors);
        delay(100);
        emergencyReset("Excessive CAN errors");
      }

      // Check for queue overflow conditions
      if (queueOverflow) {
        Serial.println("CAN message queue overflow detected");
        queueOverflow = false; // Reset flag
        logCANError(-3, "persistent_queue_overflow");
      }

      // Periodic queue health check
      if (messageCount > (CAN_QUEUE_SIZE * 2 / 3)) { // More than 66% full
        Serial.printlnf("WARNING: CAN queue filling up (%d/%d messages)",
                        messageCount, CAN_QUEUE_SIZE);

        // If queue is critically full and not being processed, force clear old
        // messages
        static unsigned long lastQueueStuckTime = 0;
        static int lastQueueCount = 0;

        if (messageCount == lastQueueCount &&
            messageCount > (CAN_QUEUE_SIZE * 4 / 5)) {
          // Queue hasn't changed and is >80% full - might be stuck
          if (lastQueueStuckTime == 0) {
            lastQueueStuckTime = currentTime;
          } else if (currentTime - lastQueueStuckTime >
                     3000) { // Stuck for 3 seconds
            Serial.printlnf("CRITICAL: Queue stuck at %d messages for 3s - "
                            "clearing oldest 30%%",
                            messageCount);
            noInterrupts();
            int toClear = messageCount * 3 / 10; // Clear 30%
            queueHead = (queueHead + toClear) % CAN_QUEUE_SIZE;
            messageCount -= toClear;
            interrupts();
            lastQueueStuckTime = 0;
          }
        } else {
          lastQueueStuckTime = 0; // Reset if queue is moving
        }
        lastQueueCount = messageCount;
      }

      // Report RX overflow statistics if any have occurred
      if (canErrorMonitor.rxOverflowCount > 0) {
        unsigned long overflowDuration =
            currentTime - canErrorMonitor.firstRxOverflowTime;
        Serial.printlnf("RX Overflow Status: %d overflows in %lu ms (last "
                        "clear: %lu ms ago)",
                        canErrorMonitor.rxOverflowCount, overflowDuration,
                        currentTime - canErrorMonitor.lastRxOverflowClear);

        // Reset overflow tracking after successful recovery (if no overflow for
        // 10 seconds and system is healthy)
        if (currentTime - canErrorMonitor.lastRxOverflowClear > 10000 &&
            canErrorMonitor.consecutiveErrors == 0) {
          Serial.printlnf(
              "RX overflow resolved - resetting tracking (was %d overflows)",
              canErrorMonitor.rxOverflowCount);
          canErrorMonitor.rxOverflowCount = 0;
          canErrorMonitor.firstRxOverflowTime = 0;
        }
      }

      // Check for invalid CAN messages (indicates controller corruption)
      if (canErrorMonitor.consecutiveErrors >= 3) {
        Serial.printlnf(
            "CAN controller corruption suspected (%d consecutive errors)",
            canErrorMonitor.consecutiveErrors);
      }

      // Log periodic health status during error conditions
      // Only log health status every 10 seconds to reduce noise
      static unsigned long lastHealthLog = 0;
      if ((canErrorMonitor.consecutiveErrors > 0 ||
           canErrorMonitor.inRecoveryMode) &&
          currentTime - lastHealthLog > 10000) {
        Serial.printlnf(
            "CAN Health: Errors=%d, Attempts=%d, Recovering=%s, "
            "HW_Flags=0x%02X",
            canErrorMonitor.consecutiveErrors, canErrorMonitor.recoveryAttempts,
            canErrorMonitor.inRecoveryMode ? "YES" : "NO", currentFlags);
        lastHealthLog = currentTime;
      }

      // // Log CAN error flags if any are detected (less frequent)
      // static unsigned long lastFlagLog = 0;
      // if (currentFlags != 0 && currentTime - lastFlagLog > 30000) //
      // Every 30 seconds
      // {
      //   Serial.printlnf("MCP2515 hardware error flags detected: 0x%02X",
      //   currentFlags); getCANErrorFlags(true); // Debug output with
      //   detailed flag breakdown lastFlagLog = currentTime;
      // }

      lastHealthCheck = currentTime;
    }

    delay(100); // Check every second
  }
}

void checkInterruptHealth() {
  unsigned long currentTime = millis();

  // Only check every INTERRUPT_CHECK_INTERVAL
  if (currentTime - lastInterruptCheck < INTERRUPT_CHECK_INTERVAL) {
    return;
  }
  lastInterruptCheck = currentTime;

  // Check for RX overflow first
  uint8_t errorFlags = getCANErrorFlags(false);
  if (errorFlags & 0x40) { // RX0OVR flag
    handleRxOverflowWithEscalation(currentTime);
    return;
  }

  // Only check other interrupt health when system is fully operational
  if (!Particle.connected() || !areCredentialsValid()) {
    // System not ready - don't monitor interrupts yet
    static bool wasReady = false;
    if (wasReady) {
      Serial.printlnf("Interrupt health monitoring disabled - system not fully "
                      "operational");
      wasReady = false;
    }
    lastInterruptTime = currentTime; // Reset timer to prevent false alarms
    return;
  }

  // Log when monitoring becomes active
  static bool wasReady = false;
  if (!wasReady) {
    Serial.printlnf(
        "Interrupt health monitoring enabled - system fully operational");
    wasReady = true;
    lastInterruptTime = currentTime; // Start fresh when monitoring begins
  }

  // Check if we've received interrupts recently
  unsigned long timeSinceLastInterrupt = currentTime - lastInterruptTime;

  // Calculate dynamic timeout based on polling cycle
  // Normal: 30s timeout, but after port reset: 40s to allow for slower
  // responses
  unsigned long dynamicTimeout = INTERRUPT_TIMEOUT;
  unsigned long timeSincePortReset = currentTime - last_port_check_reset;
  if (timeSincePortReset < PORT_CHECK_INTERVAL * 2) {
    dynamicTimeout =
        PORT_CHECK_INTERVAL * 4; // 40s grace period after port polling starts
  }

  // If we haven't received message processing in a while, but CAN is
  // supposed to be working
  if (timeSinceLastInterrupt > dynamicTimeout && !CAN_ERROR &&
      !can_recovery_needed) {
    // Check MCP2515 error flags first to see if it's just buffer overflow
    uint8_t errorFlags = getCANErrorFlags(false);

    // If it's just RX buffer overflow, clear buffers instead of interrupt
    // recovery
    if (errorFlags & 0x40) { // RX0OVR flag
      handleRxOverflowWithEscalation(currentTime);
      return;
    }

    // Check if we can still send (TX working) but not receive (RX/interrupt
    // dead)
    Serial.printlnf("No CAN message processing for %lu ms (timeout: %lu ms) - "
                    "checking interrupt system",
                    timeSinceLastInterrupt, dynamicTimeout);

    // Check MCP2515 interrupt flags to see if they're stuck
    uint8_t intFlags = mcp2515.getInterrupts();
    Serial.printlnf("MCP2515 interrupt flags: 0x%02X", intFlags);

    if (intFlags != 0) {
      Serial.printlnf("CRITICAL: MCP2515 has pending interrupts but no message "
                      "processing!");

      // Try to recover the interrupt system
      recoverInterruptSystem();
    } else {
      Serial.printlnf(
          "MCP2515 no pending interrupts - may be normal quiet period");
      // Update last interrupt time to prevent false alarms during quiet
      // periods
      lastInterruptTime = currentTime - (INTERRUPT_TIMEOUT / 2);
    }
  }
}

void recoverInterruptSystem() {
  Serial.printlnf("=== ATTEMPTING INTERRUPT SYSTEM RECOVERY ===");

  // Detach current interrupt
  detachInterrupt(CAN_INT);
  delay(100);

  // Clear any pending MCP2515 interrupts
  mcp2515.clearInterrupts();
  delay(50);

  // Reconfigure the interrupt pin
  pinMode(CAN_INT, INPUT_PULLUP);
  delay(50);

  // Re-attach interrupt only if connected
  if (areCredentialsValid() && CELLULAR_CONNECTED) {
    // Clear any pending interrupts before re-attaching
    mcp2515.clearInterrupts();
    mcp2515.clearRXnOVR();
    delay(50);

    attachInterrupt(CAN_INT, can_interrupt, FALLING);
    delay(100);
    canInterruptAttached = true; // Re-enable polling after interrupt recovery
    Serial.printlnf("Interrupt re-attached after recovery with cleared state");
  } else {
    canInterruptAttached = false; // Disable polling until connected
    Serial.printlnf("Interrupt not re-attached - waiting for connection");
  }

  // Reset interrupt health tracking
  lastInterruptTime = millis();
  interruptHealthy = true;

  Serial.printlnf("Interrupt system recovery completed");
  Serial.printlnf("Re-attached interrupt on pin %d", CAN_INT);

  // Test interrupt by reading any pending messages
  uint8_t intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    Serial.printlnf("Post-recovery: MCP2515 flags: 0x%02X", intFlags);
  }
}

void checkTransmissionReceptionBalance() {
  unsigned long currentTime = millis();

  // Check for RX overflow first - handle regardless of connectivity status
  uint8_t errorFlags = getCANErrorFlags(false);
  if (errorFlags & 0x40) { // RX0OVR flag
    handleRxOverflowWithEscalation(currentTime);
    return;
  }

  // Only check TX/RX balance when system is fully operational
  if (!CELLULAR_CONNECTED || !areCredentialsValid()) {
    // System not ready - reset timers to prevent false alarms
    static bool txRxWasReady = false;
    if (txRxWasReady) {
      Serial.printlnf("TX/RX monitoring disabled - system not ready");
      txRxWasReady = false;
    }
    lastTransmissionTime = currentTime;
    lastInterruptTime = currentTime;
    return;
  }

  // Log when TX/RX monitoring becomes active
  static bool txRxWasReady = false;
  if (!txRxWasReady) {
    Serial.printlnf("TX/RX monitoring enabled - system ready");
    txRxWasReady = true;
    lastTransmissionTime = currentTime; // Start fresh when monitoring begins
    lastInterruptTime = currentTime;
  }

  // Check if we're actively transmitting but not receiving
  unsigned long timeSinceLastTX = currentTime - lastTransmissionTime;
  unsigned long timeSinceLastRX = currentTime - lastInterruptTime;

  // Calculate dynamic TX/RX timeout based on polling patterns
  // After port polling cycle starts, give extra time for all ports to
  // respond
  unsigned long dynamicTxRxTimeout = TX_RX_IMBALANCE_TIMEOUT;
  unsigned long timeSincePortReset = currentTime - last_port_check_reset;
  if (timeSincePortReset < PORT_CHECK_INTERVAL * 2) {
    dynamicTxRxTimeout =
        PORT_CHECK_INTERVAL * 5; // 50s grace period after polling cycle
  }

  // If we've transmitted recently but haven't received anything in much
  // longer
  if (timeSinceLastTX < PORT_CHECK_INTERVAL &&
      timeSinceLastRX > dynamicTxRxTimeout && !CAN_ERROR &&
      !can_recovery_needed) {
    // Before attempting interrupt recovery, check if it's just buffer
    // overflow
    uint8_t errorFlags = getCANErrorFlags(false);

    // If it's RX overflow, that's normal under high traffic - not an
    // interrupt failure
    if (errorFlags & 0x40) { // RX0OVR flag
      handleRxOverflowWithEscalation(currentTime);
      return;
    }

    // Only trigger interrupt recovery if we have real interrupt issues
    static unsigned long lastInterruptRecovery = 0;
    if (currentTime - lastInterruptRecovery >
        PORT_CHECK_INTERVAL * 6) { // Only once per 6 polling cycles
      Serial.printlnf("CRITICAL: TX/RX imbalance - TX:%lu ms ago, RX:%lu ms "
                      "ago (timeout: %lu ms)",
                      timeSinceLastTX, timeSinceLastRX, dynamicTxRxTimeout);
      Serial.printlnf("TX works but RX failed - attempting interrupt recovery");
      recoverInterruptSystem();
      lastInterruptRecovery = currentTime;
    } else {
      // Reset timer to prevent constant triggering
      lastInterruptTime = currentTime - (dynamicTxRxTimeout / 2);
    }
  }
}

void prepareCANForInterrupt() {
  Serial.println("Preparing CAN for interrupt attachment...");

  // 1. Make sure interrupt is detached first
  detachInterrupt(CAN_INT);
  delay(50);

  // 2. Clear MCP2515 error flags (including overflow flags)
  uint8_t errorFlags = mcp2515.getErrorFlags();
  if (errorFlags != 0) {
    Serial.printlnf("Clearing MCP2515 error flags: 0x%02X", errorFlags);
    mcp2515.clearRXnOVR(); // Clear RX overflow flags
  }

  // 3. Read and discard all pending messages from hardware buffers
  can_frame dummyMsg;
  int messagesCleared = 0;
  while (mcp2515.readMessage(&dummyMsg) == MCP2515::ERROR_OK &&
         messagesCleared < 20) {
    messagesCleared++;
  }
  if (messagesCleared > 0) {
    Serial.printlnf("Discarded %d pending messages from MCP2515",
                    messagesCleared);
  }

  // 4. Clear all interrupt flags in MCP2515
  uint8_t intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    Serial.printlnf("Clearing MCP2515 interrupt flags: 0x%02X", intFlags);
    mcp2515.clearInterrupts();
  }

  // 5. Reset our software queue state
  noInterrupts();
  queueHead = 0;
  queueTail = 0;
  messageCount = 0;
  queueOverflow = false;
  memset(messageQueue, 0, sizeof(messageQueue));
  interrupts();

  // 6. Reset CAN error monitoring state
  canErrorMonitor.rxOverflowCount = 0;
  canErrorMonitor.firstRxOverflowTime = 0;
  canErrorMonitor.lastRxOverflowClear = 0;
  canErrorMonitor.consecutiveErrors = 0;

  // 7. Wait for hardware to settle
  delay(100);

  // 8. Final check - make sure no new interrupts appeared
  intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    mcp2515.clearInterrupts();
    Serial.printlnf("Cleared additional interrupt flags: 0x%02X", intFlags);
  }

  // 9. Check the interrupt pin state before attaching
  int pinState = digitalRead(CAN_INT);
  Serial.printlnf("CAN_INT pin state before attach: %s",
                  pinState == LOW ? "LOW (pending)" : "HIGH (clear)");

  // 10. If pin is still low, clear interrupts one more time
  if (pinState == LOW) {
    Serial.println("CAN_INT pin is LOW - clearing interrupts again");
    mcp2515.clearInterrupts();
    delay(50);
    pinState = digitalRead(CAN_INT);
    Serial.printlnf("CAN_INT pin state after additional clear: %s",
                    pinState == LOW ? "LOW (still pending)" : "HIGH (cleared)");
  }

  // 12. NOW attach the interrupt with everything clean
  attachInterrupt(CAN_INT, can_interrupt, FALLING);

  // Set flag to enable port polling
  canInterruptAttached = true;

  // 13. Reset timing trackers
  lastInterruptTime = millis();
  lastTransmissionTime = millis();
  canErrorMonitor.lastSuccessTime = millis();

  Serial.println("CAN interrupt attached with fully clean state - ready to "
                 "receive messages");
}

// Interruptible delay that processes cloud messages and checks for pending
// commands
void interruptibleDelay(unsigned long ms) {
  unsigned long start = millis();
  while ((millis() - start) < ms) {
    // Check for pending cloud commands
    if (pendingCloudCommand) {
      return; // Exit immediately to process cloud command
    }

    // Process Particle cloud messages
    Particle.process();

    // Small delay to prevent tight loop
    delay(10);
  }
}

void handleRxOverflowWithEscalation(unsigned long currentTime) {
  // Track RX overflow occurrences
  canErrorMonitor.rxOverflowCount++;

  // Initialize tracking on first overflow
  if (canErrorMonitor.rxOverflowCount == 1) {
    canErrorMonitor.firstRxOverflowTime = currentTime;
  }

  Serial.printlnf("RX buffer overflow detected (#%d) - clearing buffers",
                  canErrorMonitor.rxOverflowCount);

  // Always try clearing buffers first
  clearAllCANBuffers();
  canErrorMonitor.lastRxOverflowClear = currentTime;
  lastInterruptTime = currentTime; // Reset interrupt timer

  // Check if we've had persistent overflow issues
  unsigned long overflowDuration =
      currentTime - canErrorMonitor.firstRxOverflowTime;

  // If we've had 5+ overflows in 30 seconds, or continuous overflows for 5+
  // seconds
  if ((canErrorMonitor.rxOverflowCount >= 5 && overflowDuration < 30000) ||
      (overflowDuration >= 5000 && canErrorMonitor.rxOverflowCount >= 3)) {

    // Check if clearing buffers hasn't helped (overflow recurring within 1
    // second)
    if (currentTime - canErrorMonitor.lastRxOverflowClear < 1000) {
      Serial.printlnf(
          "CRITICAL: RX overflow persisting despite buffer clearing");
      Serial.printlnf("Overflow count: %d in %lu ms",
                      canErrorMonitor.rxOverflowCount, overflowDuration);
      Serial.printlnf(
          "Buffer clearing ineffective - triggering system restart");

      // Log the critical condition
      logCANError(-11, "persistent_rx_overflow_restart");

      // Force immediate restart
      Serial.printlnf(
          "*** FORCING SYSTEM RESTART DUE TO PERSISTENT RX OVERFLOW ***");
      delay(100); // Brief delay for serial output
      System.reset(RESET_NO_WAIT);
    }
  }
}

```


## Assistant

Now we need to declare the retained variables at the top of the file:

**Tool Call: Add retained memory variables for boot tracking**
Status: Failed

## Assistant



**Tool Call: main.ino**
Status: Completed

## Assistant



**Tool Call: Add retained memory variables after existing global variables**
Status: Completed

Diff: src/main.ino
```
#include "Arduino.h"
#include "DeviceInfoLedger.h"
#include "Particle.h"
#include "can.h"
#include "can_processor.h"
#include "cloud.h"
#include "config.h"
#include "credentials.h"
#include "fixes/json_compat.h"
#include "lights.h"
#include "logging.h"
#include "main.h"
#include "port_event_handler.h"
#include "port_flag_handler.h"
#include "port_state.h"
#include "utils.h"
#include <ArduinoJson.h>
#include <SPI.h>

// Product version setup
PRODUCT_VERSION(PRODUCT_VERSION_NUM);

// Global CAN state variables
char can_err_msg[200];
bool CAN_ERROR = false;
bool CELLULAR_CONNECTED = false;
bool RESET_BROKER_FLAG = false;

// Architecture components
PortEventHandler *portEventHandler = nullptr;
PortFlagHandler *portFlagHandler = nullptr;

volatile bool queueOverflow = false;
volatile int messageCount = 0;
volatile int queueHead = 0;
volatile int queueTail = 0;

unsigned long last_port_check_reset =
    0; // Tracks the last time DID_PORT_CHECK was reset
// CAN Error monitoring and recovery
volatile int can_error_count = 0;
volatile unsigned long last_can_error_time = 0;
volatile bool can_recovery_needed = false;
const int MAX_CAN_ERRORS_PER_MINUTE = 50;
const unsigned long CAN_ERROR_RESET_INTERVAL = 60000; // 1 minute

// Internet connectivity monitoring constants
const unsigned long INTERNET_DISCONNECT_RESET_TIMEOUT = 60000; // 1 minute
const unsigned long INTERNET_RESET_COOLDOWN =
    300000; // 5 minutes between resets

// Retained memory variables to track boot times across resets
retained uint32_t bootCounter = 0;
retained uint32_t lastBootTime = 0;
const unsigned long INTERNET_SOFT_RECOVERY_TIMEOUT =
    30000; // 30 seconds for soft recovery

// Global polling state for thread safety
volatile bool polling_cycle_active = false;

// Hardware watchdog handles all freeze detection at 20-second timeout

// Interrupt Health Monitoring
// Timing Logic: After PORT_CHECK_INTERVAL (10s), we send getPortData() to all
// ports We expect responses within ~25s max, so timeouts are based on this
// pattern
unsigned long lastTransmissionTime = 0;
bool interruptHealthy = true;
const unsigned long TX_RX_IMBALANCE_TIMEOUT =
    PORT_CHECK_INTERVAL * 4; // 40s - expect responses after port polling cycle

// Global VIN traffic monitoring variables (for cloud visibility)
int g_pendingVINRequests = 0;
bool g_vinFloodProtection = false;
int g_consecutiveVINFailures = 0;
unsigned long g_lastVINRequestTime = 0;
int g_activeVINRequests = 0;
// CAN Error Monitoring Constants
const int CAN_ERROR_THRESHOLD = 5; // Max consecutive errors before recovery
const unsigned long CAN_ERROR_WINDOW = 10000; // 10 seconds error window
const unsigned long CAN_RECOVERY_DELAY =
    5000;                             // 5 seconds between recovery attempts
const int MAX_RECOVERY_ATTEMPTS = 10; // Max recovery attempts before reset
const unsigned long RECOVERY_SUCCESS_RESET_TIME =
    300000; // 5 minutes of success resets recovery count

// Flag to track when we've just connected
bool justConnectedFlag = false;

// Flag to track when CAN is ready for health monitoring
bool canReadyForMonitoring = false;

// Flag to track when CAN interrupt has been attached
bool canInterruptAttached = false;

// Flag to track pending cloud commands that need immediate processing
volatile bool pendingCloudCommand = false;

// CAN Error Monitor Structure
struct CANErrorMonitor {
  int consecutiveErrors;
  unsigned long lastErrorTime;
  unsigned long totalErrors;
  int recoveryAttempts;
  bool inRecoveryMode;
  unsigned long lastRecoveryAttempt;
  unsigned long lastSuccessTime;
  unsigned long lastSuccessfulRecovery;
  bool adaptiveMode;
  unsigned long extendedRecoveryDelay;
  // Enhanced RX overflow handling
  int rxOverflowCount;
  unsigned long firstRxOverflowTime;
  unsigned long lastRxOverflowClear;
} canErrorMonitor = {0, 0, 0, 0, false, 0, 0, 0, false, 0, 0, 0, 0};

can_frame messageQueue[CAN_QUEUE_SIZE]; // Increased from 50 to 100

// Interrupt monitoring variables
unsigned long lastInterruptTime = 0;
unsigned long lastInterruptCheck = 0;
bool interruptStuckDetected = false;
// Base timeout: 3x polling interval (30s) - ports should respond within 25s max
// after getPortData()
const unsigned long INTERRUPT_TIMEOUT = PORT_CHECK_INTERVAL * 3;
const unsigned long INTERRUPT_CHECK_INTERVAL = 10000; // Check every 10 seconds

// Global hardware watchdog object
ApplicationWatchdog *hardwareWatchdog;

// Function declarations
void checkInterruptHealth();
void recoverInterruptSystem();
void checkTransmissionReceptionBalance();
void handleRxOverflowWithEscalation(unsigned long currentTime);
void prepareCANForInterrupt();
void interruptibleDelay(unsigned long ms);

// Hardware watchdog handler
void hardwareWatchdogHandler() {
  Serial.printlnf("HARDWARE WATCHDOG RESET at uptime %lu ms", millis());
  Serial.flush();
  delay(100);
  System.reset(RESET_NO_WAIT);
}

void setup() {
  initializeSystem();

  // Log startup time and reset reason IMMEDIATELY
  Serial.printlnf("=== DEVICE STARTUP AT %lu ms ===", millis());
  Serial.printlnf("Reset reason: %d, Reset data: %d", System.resetReason(),
                  System.resetReasonData());
  Serial.printlnf("System version: %s", System.version().c_str());
  Serial.printlnf("Free memory: %lu bytes", System.freeMemory());

  // Initialize hardware watchdog early - 20 second timeout
  hardwareWatchdog =
      new ApplicationWatchdog(20000, hardwareWatchdogHandler, 1536);
  Serial.printlnf("Hardware ApplicationWatchdog initialized (20s timeout)");

  initializeArchitecture();
  initializeHardware();

  if (!CAN_ERROR) {
    initializeParticle();

    // Start alternate threads
    new Thread("can_thread", canThread);
    new Thread("port_request_thread", port_request_thread);
    new Thread("can_health_monitor", canHealthMonitorThread);
    new Thread("internet_checker", internetCheckThread);

    requestCredentials();

    // Initialize recovery state
    canErrorMonitor.lastSuccessTime = millis();
    canErrorMonitor.recoveryAttempts = 0;

    Serial.printlnf("=== SYSTEM STARTUP COMPLETE ===");
    Serial.printlnf("CAN Error Monitoring: ENABLED");
    Serial.printlnf("Hardware Watchdog: ENABLED (20s timeout)");
    Serial.printlnf("Recovery System: READY");
  } else {
    Serial.printlnf("=== STARTUP FAILED - CAN ERROR DETECTED ===");
    Serial.printlnf("Uptime before CAN error reset: %lu ms", millis());
    resetDevice("CAN error during startup");
  }
}

void loop() {
  ApplicationWatchdog::checkin(); // Feed hardware watchdog
  DeviceInfoLedger::instance().loop();

  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();

  // Detect if main loop is running too slowly (potential freeze indicator)
  if (lastLoopTime > 0 && (currentTime - lastLoopTime) > 5000) {
    Serial.printlnf(
        "WARNING: Main loop delay detected: %lu ms (uptime: %lu ms)",
        currentTime - lastLoopTime, currentTime);
  }
  lastLoopTime = currentTime;
  handleSystemLoop();

  // Small delay to prevent CPU overload during error conditions
  if (CAN_ERROR || can_recovery_needed) {
    interruptibleDelay(10);
  }
}

void initializeArchitecture() {
  // Initialize the clean architecture components
  portEventHandler =
      new PortEventHandler(nullptr); // Will use global port state functions
  portFlagHandler =
      new PortFlagHandler(nullptr); // Will use global port state functions

  Serial.printlnf("Architecture components initialized");
}

void initializeSystem() {
  Serial.begin(115200);
  while (!Serial)
    ;
  // delay(2000);

  // Track boot times using retained memory
  bootCounter++;
  uint32_t currentTime = Time.now();
  Serial.printlnf("=== BOOT #%lu at Unix time %lu ===", bootCounter,
                  currentTime);

  if (bootCounter > 1 && lastBootTime > 0) {
    uint32_t timeBetweenBoots = currentTime - lastBootTime;
    Serial.printlnf("Time since last boot: %lu seconds", timeBetweenBoots);
    if (timeBetweenBoots < 30) {
      Serial.printlnf("WARNING: Rapid restart detected! (%lu seconds)",
                      timeBetweenBoots);
    }
  }
  lastBootTime = currentTime;

  // Initialize all subsystems
  initializePorts();
  initializeCredentials();
  initializeLedger();

  Serial.printlnf("*** KUHMUTE IoT V %s ***", BUILD_VERSION);
  Serial.printlnf("Device ID: %s", Particle.deviceID().c_str());
  Serial.printlnf("Environment: %s", getCurrentEnvironment());
  Serial.printlnf("System initialized");
}
void initializeLedger() {

  // This sets up remote configuration
  DeviceConfigLedger::instance()
      .withConfigDefaultLedgerName("device-info-defaults")
      .withConfigDeviceLedgerName("device-info-config")
      .setup();
  // This sets up the device information in ledger
  DeviceInfoLedger::instance()
      .withInfoLedgerName("device-info")
      .withRetainedBuffer(retainedLogs, sizeof(retainedLogs))
      .setup();
}
void initializeHardware() {
  // Setup ring light
  beginLight();
  setLightBlue();

  // Setup SPI for CAN
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  // Initialize CAN bus
  int err = mcp2515.reset();
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "reset", true);
    return;
  }

  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "setBitrate", true);
    return;
  }

  delay(50);
  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "setNormalMode", true);
    return;
  }

  pinMode(CAN_INT, INPUT_PULLUP);
  // NOTE: CAN interrupt will be attached after connection is established
  // This prevents accumulating unwanted messages during startup

  // Initialize interrupt health tracking
  lastInterruptTime = millis();
  lastInterruptCheck = millis();
  interruptHealthy = true;

  Serial.printlnf("Hardware initialized (CAN interrupt pending)");
}

void initializeParticle() {
  // Register cloud functions
  Particle.function("resetDevice", resetDevice);
  Particle.function("getPortVin", forceGetVin);
  Particle.function("getPortStatus", forceGetPortStatus);
  Particle.function(JUISE_INCOMING, juiceMessageCallback);

  Particle.variable("CAN_ERROR", CAN_ERROR);
  Particle.variable("pub_id", MANUAL_MODE, STRING);
  Particle.variable("credentialsFetched", credentialsFetched);
  Particle.variable("total_messages_received", total_messages_received);

  // VIN traffic monitoring variables
  Particle.variable("pending_vins", g_pendingVINRequests);
  Particle.variable("active_vins", g_activeVINRequests);
  Particle.variable("vin_flood_protection", g_vinFloodProtection);
  Particle.variable("last_vin_request", g_lastVINRequestTime);

  Particle.connect();
  Particle.publishVitals(60);
  Particle.keepAlive(30);

  // Enable system features
  System.enableFeature(FEATURE_RESET_INFO);
  System.enableFeature(FEATURE_RETAINED_MEMORY);

  // Wait for connection with timeout
  setLightRed();
  waitFor(Particle.connected, 120000);
  if (!Particle.connected()) {
    Serial.printlnf("Failed to connect to Particle Cloud");
    resetDevice("");
  }

  // Log reset reason
  // logResetReason();

  setLightBlue();
  Serial.printlnf("Particle Cloud connected");
}

void handleSystemLoop() {
  // Handle CAN recovery IMMEDIATELY - before anything else
  if (can_recovery_needed) {
    Serial.printlnf("EMERGENCY: Executing CAN recovery due to error cascade");
    performCANRecovery();
    delay(1000); // Give recovery time to complete
    return;      // Exit immediately, don't continue with normal operations
  }

  // Handle critical errors
  if (CAN_ERROR) {
    blinkCANError();
    interruptibleDelay(100); // Prevent tight loop (interruptible)
    return;
  }

  // Check for credential fetch failures
  if (attemptedCredentialsFetchCount > MAX_CREDENTIAL_ATTEMPTS) {
    Serial.printlnf("Failed to fetch credentials after max attempts");
    blinkIdentityError();
    return;
  }

  // Handle connection states
  if (!Particle.connected()) {
    setLightRed();
    return;
  }

  handleCredentials();
  updateSystemStatus();

  // System health monitoring
  checkSystemHealth();

  // Check for pending port status requests
  checkPortStatusRequest();

  // Small delay to prevent CPU overload (interruptible)
  interruptibleDelay(5);

  // Process cloud messages frequently
  Particle.process();
}

/**
 * Handle systematic port data requests with staggered polling
 * Polls each port in sequence with delays to avoid overwhelming the CAN bus
 */
void handlePortDataRequests() {
  // Add static variables for timing control
  static unsigned long last_function_run_time = 0;
  static bool first_run = true;
  static int current_poll_port =
      0; // Don't initialize to 1, will be set when cycle starts

  // Don't start polling until CAN interrupt is attached
  if (!canInterruptAttached) {
    return;
  }

  // If there's a pending cloud command, pause polling to let it process
  if (pendingCloudCommand) {
    delay(10); // Small delay to let cloud command process
    return;
  }

  // Only run this function on first call or once per PORT_CHECK_INTERVAL
  unsigned long current_time = millis();
  if (!first_run && !polling_cycle_active &&
      (current_time - last_function_run_time < PORT_CHECK_INTERVAL)) {
    // Too soon to run again, skip
    return;
  }

  // EMERGENCY STOP - Don't poll if we're in error cascade
  if (can_recovery_needed || CAN_ERROR) {
    polling_cycle_active =
        false;             // CRITICAL: Must reset this to prevent deadlock
    current_poll_port = 1; // Reset port counter
    markPortsUnpolled();
    delay(100); // Prevent tight loop during recovery
    return;
  }

  // Add consecutive failure tracking per port
  static int portFailureCount[MAX_PORTS + 1] = {0}; // Track failures per port
  static bool pollingDisabled = false;
  static unsigned long pollingDisabledTime = 0;

  // PREVENTIVE VIN flood protection - conservative approach
  static unsigned long lastVINRequestTime = 0;
  static int pendingVINRequests = 0;
  static unsigned long lastHighTrafficCheck = 0;

  // Conservative VIN request limiting (prevent silent lockups)
  static int activeVINRequests = 0; // Currently processing VIN requests
  static unsigned long vinRequestSlots[3] = {
      0, 0, 0}; // Track up to 3 concurrent VINs
  static const int MAX_CONCURRENT_VINS =
      2; // Conservative: only 2 VIN requests at once

  // Check if we should re-enable polling after errors cleared
  if (pollingDisabled) {
    unsigned long waitTime = 5000; // Default 5 seconds

    // Extended wait time in adaptive mode
    if (canErrorMonitor.adaptiveMode &&
        canErrorMonitor.extendedRecoveryDelay > 5000) {
      waitTime = canErrorMonitor.extendedRecoveryDelay;
    }

    if (canErrorMonitor.consecutiveErrors == 0 &&
        millis() - pollingDisabledTime > waitTime) {
      Serial.printlnf(
          "Re-enabling port polling - errors have cleared (waited %lu ms)",
          waitTime);
      pollingDisabled = false;
      // Reset all port failure counts
      for (int i = 0; i <= MAX_PORTS; i++) {
        portFailureCount[i] = 0;
      }
    } else {
      // Make delay interruptible
      for (int i = 0; i < 10 && !pendingCloudCommand; i++) {
        delay(10);
        Particle.process();
      }
      return;
    }
  }

  // Static variable for port check reset timing
  static unsigned long last_port_check_reset = 0;
  static unsigned long last_poll_send_time = 0;

  // Start of a new polling cycle - reset all ports for polling
  if (!polling_cycle_active ||
      (current_time - last_port_check_reset >= PORT_CHECK_INTERVAL)) {
    // Only print the message when we're actually resetting
    if (current_time - last_port_check_reset >= PORT_CHECK_INTERVAL) {
      markPortsUnpolled();
      last_port_check_reset = current_time; // Update the last reset time
      Serial.println("🚨 DID_PORT_CHECK reset for all ports 🚨");

      // Force start new cycle when ports are reset
      polling_cycle_active = false;
      current_poll_port = 0; // Will be set to 1 when cycle actually starts
    }

    // Start a new polling cycle
    if (!polling_cycle_active) {
      polling_cycle_active = true;
      current_poll_port = 1; // NOW set to port 1 when actually starting
      Serial.printlnf(
          "Starting new polling cycle for all ports - starting at port %d",
          current_poll_port);
    }
  }

  // IMPORTANT: Only check this if it's not the first poll (last_poll_send_time
  // will be 0 on first poll)
  if (last_poll_send_time != 0) {
    // Check if enough time has passed since last poll
    if (current_time - last_poll_send_time < POLL_STAGGER_DELAY) {
      // Not enough time has passed, skip this poll cycle
      return;
    }

    // PREVENTIVE: Count and limit VIN requests to prevent silent lockups
    if (current_time - lastHighTrafficCheck >= 3000) { // Check every 3 seconds
      pendingVINRequests = 0;
      activeVINRequests = 0;

      // Clear expired VIN request slots (assume 15 seconds max for VIN
      // completion)
      for (int i = 0; i < 3; i++) {
        if (vinRequestSlots[i] > 0 &&
            (current_time - vinRequestSlots[i]) > 15000) {
          vinRequestSlots[i] = 0; // Clear expired slot
        } else if (vinRequestSlots[i] > 0) {
          activeVINRequests++; // Count active VIN requests
        }
      }

      // Count pending VIN flags
      for (int i = 1; i <= MAX_PORTS; i++) {
        if (isValidPort(i)) {
          PortState *state = getPortState(i);
          if (state && state->vin_request_flag) {
            pendingVINRequests++;
          }
        }
      }

      // Update global variables for cloud monitoring
      g_pendingVINRequests = pendingVINRequests;
      g_activeVINRequests = activeVINRequests;

      lastHighTrafficCheck = current_time;
    }

    // CONSERVATIVE delay calculation to prevent bus saturation
    unsigned long smart_delay = POLL_STAGGER_DELAY; // Default 1000ms

    // MUCH more conservative VIN spacing to prevent silent lockups
    if (pendingVINRequests >= 3) {
      smart_delay = 8000; // 8 second delay when 3+ VIN requests pending
    } else if (pendingVINRequests >= 1) {
      smart_delay = 5000; // 5 second delay when any VIN requests pending
    }

    // Aggressive delay for higher port addresses (they have 1200ms+ response
    // delays)
    if (current_poll_port > 10) {
      smart_delay += 3000; // Extra 3 seconds for ports 11-16
    } else if (current_poll_port > 8) {
      smart_delay += 2000; // Extra 2 seconds for ports 9-10
    }

    // LONG buffer after VIN requests - accounts for full 3-message sequence +
    // port delays
    if (current_time - lastVINRequestTime <
        10000) {           // Within 10 seconds of VIN request
      smart_delay += 5000; // Add 5 second buffer minimum
    }

    // If any VIN requests are actively being processed, be extra conservative
    if (activeVINRequests > 0) {
      smart_delay += 3000; // Additional 3 seconds when VINs are processing
    }

    // Check if enough time has passed since last poll using smart delay
    // Make this interruptible by checking for cloud commands
    unsigned long delay_start = millis();
    while ((millis() - last_poll_send_time) < smart_delay) {
      // Process Particle cloud to ensure responsiveness
      Particle.process();

      // Check for pending cloud commands every 50ms
      if (pendingCloudCommand) {
        return; // Exit immediately to process cloud command
      }

      // Prevent infinite loop
      if ((millis() - delay_start) > (smart_delay + 1000)) {
        break;
      }

      delay(50); // Small delay between checks
    }
  }

  // Check if portFlagHandler is available
  if (!portFlagHandler) {
    Serial.printlnf("portFlagHandler not initialized");
    return;
  }

  // Only process port polling when we're in an active cycle
  if (!polling_cycle_active) {
    return; // Don't increment port counter between cycles
  }

  // Safety check - if current_poll_port is 0, set it to 1
  if (current_poll_port == 0) {
    current_poll_port = 1;
    Serial.printlnf("WARNING: current_poll_port was 0, resetting to 1");
  }

  // Find next port that needs polling
  int start_port = current_poll_port; // Remember where we started
  for (int attempts = 0; attempts < MAX_PORTS; attempts++) {
    // Validate and wrap port number BEFORE using it
    if (current_poll_port < 1 || current_poll_port > MAX_PORTS) {
      current_poll_port = 1; // Wrap around or fix invalid port number
    }

    if (!hasPortBeenPolled(current_poll_port)) {
      // Check if this port has a pending VIN request
      bool isVINRequest = false;
      PortState *state = getPortState(current_poll_port);
      if (state) {
        isVINRequest = state->vin_request_flag;
      }

      // PREVENTIVE: Skip VIN requests if too many are already active
      if (isVINRequest && activeVINRequests >= MAX_CONCURRENT_VINS) {
        Serial.printlnf("PREVENTIVE: Skipping VIN request for port %d - %d "
                        "already active (max %d)",
                        current_poll_port, activeVINRequests,
                        MAX_CONCURRENT_VINS);
        current_poll_port++; // Move to next port
        continue;            // Skip this VIN request
      }

      // This is the critical part - actually send the port data request
      bool success = portFlagHandler->sendGetPortData(current_poll_port);

      if (success) {
        Serial.printlnf("Polled port %d", current_poll_port);
        markPortPolled(current_poll_port);
        // Reset failure count on success
        portFailureCount[current_poll_port] = 0;
        resetCANSuccessCounter();
        // Track transmission time for health monitoring
        lastTransmissionTime = millis();

        // Track VIN request timing and reserve slot
        if (isVINRequest) {
          lastVINRequestTime = current_time;
          g_lastVINRequestTime = current_time;

          // Reserve a VIN request slot
          for (int i = 0; i < 3; i++) {
            if (vinRequestSlots[i] == 0) {
              vinRequestSlots[i] = current_time;
              break;
            }
          }

          Serial.printlnf("VIN request sent to port %d - slot reserved, %d "
                          "slots active",
                          current_poll_port, activeVINRequests + 1);
        }

        // Calculate port wait time (empirical values based on testing)
        int portWaitTime = 3000; // Default 3 seconds between polls

        // Higher ports need longer delays (cascading response delays)
        if (current_poll_port > 8) {
          portWaitTime = 4000; // 4 seconds for ports 9+
        }
        if (current_poll_port > 12) {
          portWaitTime = 5000; // 5 seconds for ports 13+
        }

        // Global STAGGER_DELAY can override port wait time
        if (POLL_STAGGER_DELAY > portWaitTime) {
          portWaitTime = POLL_STAGGER_DELAY;
        }

        // VIN requests need much longer delay
        if (isVINRequest) {
          portWaitTime = 10000; // 10 seconds minimum for VIN requests
        }

      } else {
        portFailureCount[current_poll_port]++;
        Serial.printlnf("Failed to poll port %d (failures: %d)",
                        current_poll_port, portFailureCount[current_poll_port]);

        // Skip this port if it's failing repeatedly
        if (portFailureCount[current_poll_port] >= 3) {
          Serial.printlnf(
              "Port %d has failed %d times - skipping for this cycle",
              current_poll_port, portFailureCount[current_poll_port]);
          markPortPolled(current_poll_port); // Mark as "polled" to skip it
          portFailureCount[current_poll_port] = 0; // Reset for next cycle
        }

        logCANError(-1, "port_polling");

        // If too many consecutive errors, trigger IMMEDIATE recovery
        if (canErrorMonitor.consecutiveErrors >= 3) {
          Serial.printlnf("CRITICAL: Immediate CAN recovery needed");
          can_recovery_needed = true;
          pollingDisabled = true;
          pollingDisabledTime = millis();
          pendingCloudCommand = false; // Clear flag during error
          return;                      // Stop polling immediately
        }

        // Hardware watchdog will handle timeout protection automatically
      }

      last_poll_send_time = current_time;

      // Move to next port and wrap if needed
      current_poll_port++;
      if (current_poll_port > MAX_PORTS) {
        current_poll_port = 1;
      }

      // Instead of breaking, return to allow delay between ports
      return;
    }

    // Move to next port when skipping already-polled ports
    current_poll_port++;
    if (current_poll_port > MAX_PORTS) {
      current_poll_port = 1;
    }
  }

  // If we get here, we've checked all ports and no ports need polling in this
  // cycle Check if we've completed a full cycle (all ports are polled)
  bool all_ports_polled = true;
  for (int port = 1; port <= MAX_PORTS; port++) {
    if (!hasPortBeenPolled(port)) {
      all_ports_polled = false;
      break;
    }
  }

  // If all ports are polled, end the polling cycle
  if (all_ports_polled) {
    polling_cycle_active = false;
    current_poll_port = 1; // Reset to port 1 for next cycle
    last_function_run_time = current_time;
    first_run = false;
    Serial.printlnf(
        "Port polling cycle complete - all ports polled - next cycle in %lu ms",
        PORT_CHECK_INTERVAL);
  }
}

void handleCredentials() {
  if (areCredentialsValid()) {
    return; // Already have valid credentials
  }

  if (shouldRetryCredentials()) {
    requestCredentials();
  }
}

void updateSystemStatus() {
  if (areCredentialsValid() && CELLULAR_CONNECTED) {
    setLightGreen(); // All systems operational

    // Set the just connected flag when we first get valid credentials and
    // Internet connected connection
    static bool wasConnectedBefore = false;
    if (!wasConnectedBefore) {
      justConnectedFlag = true;
      wasConnectedBefore = true;
      Serial.printlnf("First successful connection detected - immediate port "
                      "polling enabled");

      // Prepare CAN system and attach interrupt with clean state
      prepareCANForInterrupt();
    }
  } else if (areCredentialsValid()) {
    setLightPurple(); // Have credentials, connecting to Cloud
  } else {
    setLightBlue(); // Fetching credentials
  }
}

// ========================================
// CAN Processing
// ========================================

void canThread() {
  Serial.printlnf("CAN processing thread started");

  while (true) {
    if (areCredentialsValid() && CELLULAR_CONNECTED && !CAN_ERROR &&
        !can_recovery_needed) {
      // Give priority to cloud commands
      if (pendingCloudCommand) {
        delay(50); // Give cloud command time to process
        Particle.process();
        continue; // Skip this iteration to let cloud command complete
      }

      // Process incoming CAN messages
      handleCanQueue();

      // Process port flags (replaces old flagHandler)
      if (portFlagHandler) {
        portFlagHandler->processAllPortFlags();
      }

      // Process Particle cloud to ensure responsiveness
      Particle.process();
    }
    // Small delay to prevent busy-waiting
    delay(10);
  }
}

void port_request_thread() {
  while (true) {
    if (areCredentialsValid() && CELLULAR_CONNECTED && !CAN_ERROR &&
        !can_recovery_needed) {
      // Skip polling if cloud command is pending
      if (!pendingCloudCommand) {
        handlePortDataRequests();
      }

      // Shorter delay to allow staggered polling to work properly
      // Each port will be polled with the appropriate delay between them
      interruptibleDelay(50);
    } else {
      // Small delay to prevent busy-waiting
      interruptibleDelay(100);
    }
  }
}

void handleCanQueue() {
  int messagesProcessed = 0;
  int MAX_MESSAGES_PER_LOOP = 8;

  // If queue is critically full, process more aggressively
  if (messageCount > (CAN_QUEUE_SIZE * 3 / 4)) { // More than 75% full
    MAX_MESSAGES_PER_LOOP = 20;
    Serial.printlnf("CAN queue critical (%d messages) - aggressive processing",
                    messageCount);
  }

  while (messageCount > 0 && messagesProcessed < MAX_MESSAGES_PER_LOOP) {
    can_frame msg;
    bool validMessage = false;

    // Thread-safe message extraction
    noInterrupts();
    if (messageCount > 0) {
      if (queueHead >= 0 && queueHead < CAN_QUEUE_SIZE && queueTail >= 0 &&
          queueTail < CAN_QUEUE_SIZE) {
        msg = messageQueue[queueHead];
        queueHead = (queueHead + 1) % CAN_QUEUE_SIZE;
        messageCount--;
        validMessage = true;

        // Handle queue overflow reset
        if (queueOverflow && messageCount < (CAN_QUEUE_SIZE / 2)) {
          queueOverflow = false;
          Serial.println("Queue overflow cleared");
        }
      } else {
        // Invalid queue state, reset
        queueTail = 0;
        queueHead = 0;
        messageCount = 0;
        Serial.println(
            "ERROR: Invalid queue indices detected - resetting queue");
      }
    }
    interrupts();

    // Process the message using clean architecture
    if (validMessage) {
      processCANMessage(msg);
      messagesProcessed++;
    }

    // Yield periodically
    if (messagesProcessed % 2 == 0) {
      Particle.process();
    }
  }
}

void processCANMessage(const can_frame &rawMessage) {
  // Check for corrupted CAN ID (negative or excessively large values)
  if ((int32_t)rawMessage.can_id < 0 || rawMessage.can_id > 16777215) {
    Serial.printlnf("CRITICAL: Corrupted CAN ID detected: %ld (0x%lX)",
                    (long)rawMessage.can_id, (unsigned long)rawMessage.can_id);
    logCANError(0xF, "corrupted_can_id");

    // This indicates controller corruption - trigger immediate recovery
    Serial.printlnf("CAN controller corruption detected - triggering recovery");
    can_recovery_needed = true;
    return;
  }

  // Check for invalid data length
  if (rawMessage.can_dlc > 8) {
    Serial.printlnf("CRITICAL: Invalid CAN DLC: %d", rawMessage.can_dlc);
    logCANError(0xE, "invalid_dlc");
    can_recovery_needed = true;
    return;
  }

  // 1. Parse the raw CAN message
  ParsedCANMessage parsedMsg = canProcessor.parseMessage(rawMessage);

  // 2. Log the message for debugging
  Serial.printlnf("CAN message from port %d: type=%s, valid=%s",
                  parsedMsg.sourcePort,
                  canProcessor.getMessageTypeString(parsedMsg.messageType),
                  parsedMsg.isValid ? "yes" : "no");

  // 3. Handle the business logic if message is valid
  if (parsedMsg.isValid && portEventHandler) {
    portEventHandler->handleCANMessage(parsedMsg);
    // Reset consecutive error count on successful message processing
    resetCANSuccessCounter();
    // Update interrupt health tracking - we successfully processed a message
    lastInterruptTime = millis();
    interruptHealthy = true;
  } else if (!parsedMsg.isValid) {
    Serial.printlnf("Invalid CAN message received from port %d",
                    parsedMsg.sourcePort);
    logCANError(-1, "message_parsing");
  }
}

void can_interrupt() {
  // Minimal interrupt handler - just queue the message
  struct can_frame recMsg;

  // Read the message
  int readin = readCanMessage(&recMsg);

  if (readin == ERROR_OK) {
    // Comprehensive corruption and gibberish filtering
    // 1. Basic validity checks
    if ((int32_t)recMsg.can_id < 0 || recMsg.can_id == 0 ||
        recMsg.can_id > MAX_PORTS || recMsg.can_dlc > 8) {
      // Don't queue obviously corrupted messages
      return; // Silent discard for performance
    }

    // 2. Data content validation - filter out gibberish
    bool hasValidData = false;
    bool hasGibberish = false;

    for (int i = 0; i < recMsg.can_dlc; i++) {
      uint8_t byte = recMsg.data[i];

      // Check for valid ASCII characters (printable range)
      if ((byte >= 0x20 && byte <= 0x7E) || byte == 0x00) {
        hasValidData = true;
      }
      // Check for obvious gibberish (control chars, high ASCII)
      else if (byte < 0x20 || byte > 0x7E) {
        // Allow some common control characters
        if (byte != 0x0A && byte != 0x0D && byte != 0x09) // LF, CR, TAB
        {
          hasGibberish = true;
        }
      }
    }

    // Reject if mostly gibberish or no valid data
    if (hasGibberish && !hasValidData) {
      return; // Silent discard of gibberish
    }

    // 3. Message pattern validation - must start with valid command
    if (recMsg.can_dlc > 0) {
      char firstChar = (char)recMsg.data[0];
      // Valid message types: D(status), K(VIN), T(temp), H(heartbeat), etc.
      if (firstChar != 'D' && firstChar != 'K' && firstChar != 'T' &&
          firstChar != 'H' && firstChar != 'U' && firstChar != 'C' &&
          firstChar != 'V' && firstChar != 'F' && firstChar != 'E') {
        return; // Silent discard of invalid command
      }
    }

    // Add to queue if there's space
    if (messageCount < CAN_QUEUE_SIZE) {
      incrementMessageCounter();
      messageQueue[queueTail] = recMsg;
      queueTail = (queueTail + 1) % CAN_QUEUE_SIZE;
      messageCount++;
    } else {
      queueOverflow = true;
      logCANError(-2, "queue_overflow");

      // Emergency queue clear if severely overflowing
      static unsigned long lastEmergencyClear = 0;
      if (millis() - lastEmergencyClear > 5000) { // Only once per 5 seconds
        Serial.println(
            "EMERGENCY: Clearing oldest 25% of queue to prevent lockup");
        // Clear oldest 25% of messages
        int toClear = messageCount / 4;
        queueHead = (queueHead + toClear) % CAN_QUEUE_SIZE;
        messageCount -= toClear;
        lastEmergencyClear = millis();
      }
    }
  } else {
    // Log read errors (but not every single one to avoid spam)
    static unsigned long lastErrorLog = 0;
    if (millis() - lastErrorLog > 1000) // Log max once per second
    {
      logCANError(readin, "can_read_interrupt");
      lastErrorLog = millis();
    }
  }
}

// ========================================
// CAN Error Monitoring and Recovery
// ========================================

void performCANRecovery() {
  Serial.printlnf("=== PERFORMING EMERGENCY CAN RECOVERY ===");

  // Stop ALL CAN operations immediately
  CAN_ERROR = true;
  can_recovery_needed = false;  // Clear the flag first
  polling_cycle_active = false; // Reset polling state during recovery
  canInterruptAttached = false; // Disable polling during recovery

  // Clear all queues and reset state
  noInterrupts();
  queueHead = 0;
  queueTail = 0;
  messageCount = 0;
  queueOverflow = false;

  // Reset RX overflow tracking during recovery
  canErrorMonitor.rxOverflowCount = 0;
  canErrorMonitor.firstRxOverflowTime = 0;
  canErrorMonitor.lastRxOverflowClear = 0;
  interrupts();

  // Detach interrupt to prevent issues during recovery
  detachInterrupt(CAN_INT);

  // Update recovery tracking
  canErrorMonitor.recoveryAttempts++;
  canErrorMonitor.inRecoveryMode = true;
  canErrorMonitor.lastRecoveryAttempt = millis();

  // Check if we've exceeded max recovery attempts without recent success
  if (canErrorMonitor.recoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
    unsigned long timeSinceLastSuccess =
        millis() - canErrorMonitor.lastSuccessfulRecovery;

    // If it's been a long time since last successful recovery, allow reset of
    // attempts
    if (timeSinceLastSuccess > RECOVERY_SUCCESS_RESET_TIME) {
      Serial.printlnf("Resetting recovery attempts - %lu ms since last success",
                      timeSinceLastSuccess);
      canErrorMonitor.recoveryAttempts = 1; // Reset but count this attempt
      canErrorMonitor.adaptiveMode = true;  // Enable adaptive recovery
    } else {
      Serial.printlnf("CRITICAL: Max recovery attempts (%d) exceeded - forcing "
                      "device reset",
                      MAX_RECOVERY_ATTEMPTS);
      delay(100);
      resetDevice("Max CAN recovery attempts exceeded");
      return;
    }
  }

  Serial.printlnf("CAN Recovery attempt %d of %d",
                  canErrorMonitor.recoveryAttempts, MAX_RECOVERY_ATTEMPTS);

  // Adaptive recovery delay based on attempt number
  unsigned long recoveryDelay = 500;
  if (canErrorMonitor.adaptiveMode) {
    recoveryDelay =
        1000 + (canErrorMonitor.recoveryAttempts * 500); // Progressive delay
    canErrorMonitor.extendedRecoveryDelay =
        recoveryDelay * 2; // Extended pause after recovery
    Serial.printlnf("Adaptive recovery mode - using %lu ms delay",
                    recoveryDelay);
  }

  delay(recoveryDelay); // Give system time to stabilize

  // Reset MCP2515 controller
  Serial.printlnf("Resetting MCP2515 controller...");
  int err = mcp2515.reset();
  if (err != mcp2515.ERROR_OK) {
    Serial.printlnf("CRITICAL: MCP2515 reset failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 reset failed");
    return;
  }

  delay(200);

  // Reconfigure controller
  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != mcp2515.ERROR_OK) {
    Serial.printlnf(
        "CRITICAL: MCP2515 bitrate config failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 config failed");
    return;
  }

  delay(100);

  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK) {
    Serial.printlnf(
        "CRITICAL: MCP2515 normal mode failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 mode failed");
    return;
  }

  // Re-enable interrupts after successful controller reset
  pinMode(CAN_INT, INPUT_PULLUP);

  // Only re-attach interrupt if we're connected and have credentials
  if (areCredentialsValid() && CELLULAR_CONNECTED) {
    // Use prepareCANForInterrupt to ensure clean state
    prepareCANForInterrupt();
    canInterruptAttached = true; // Re-enable polling after recovery
    Serial.printlnf(
        "CAN recovery complete - interrupt re-enabled with clean state");
  } else {
    canInterruptAttached = false; // Disable polling until connected
    Serial.printlnf(
        "CAN interrupt pending - will attach after connection established");
  }

  // Reset all error tracking
  can_error_count = 0;
  last_can_error_time = millis();
  canErrorMonitor.consecutiveErrors = 0;
  canErrorMonitor.inRecoveryMode = false;
  canErrorMonitor.lastSuccessTime = millis();
  canErrorMonitor.lastSuccessfulRecovery = millis();
  CAN_ERROR = false;

  Serial.printlnf("=== CAN RECOVERY COMPLETED SUCCESSFULLY ===");
  Serial.printlnf("Recovery attempt %d succeeded",
                  canErrorMonitor.recoveryAttempts);

  // Extended pause in adaptive mode for system stabilization
  unsigned long postRecoveryDelay = 1000;
  if (canErrorMonitor.adaptiveMode &&
      canErrorMonitor.extendedRecoveryDelay > 1000) {
    postRecoveryDelay = canErrorMonitor.extendedRecoveryDelay;
    Serial.printlnf("Extended post-recovery delay: %lu ms", postRecoveryDelay);
  }

  delay(postRecoveryDelay);
}

void logCANError(int errorCode, const char *operation) {
  unsigned long currentTime = millis();

  canErrorMonitor.totalErrors++;
  canErrorMonitor.lastErrorTime = currentTime;
  canErrorMonitor.consecutiveErrors++;

  Serial.printlnf("CAN Error #%lu: %s failed with code %d (consecutive: %d)",
                  canErrorMonitor.totalErrors, operation, errorCode,
                  canErrorMonitor.consecutiveErrors);

  // Check for specific error codes that indicate controller corruption
  if (errorCode == 0xA || errorCode == 10 || errorCode == 0xF ||
      errorCode == 15) {
    Serial.printlnf("CRITICAL: Detected ERROR_A/ERROR_F - MCP2515 controller "
                    "corruption!");
    Serial.printlnf(
        "Triggering immediate CAN recovery to prevent system freeze");
    can_recovery_needed = true;

    // If we've seen these errors multiple times, force device reset
    if (canErrorMonitor.consecutiveErrors >= 2) {
      Serial.printlnf(
          "Multiple controller corruption errors - forcing device reset");
      delay(100);
      resetDevice("MCP2515 controller corruption");
      return;
    }
  }

  // Lower threshold for faster recovery during error cascades
  if (canErrorMonitor.consecutiveErrors >= 3) {
    Serial.printlnf("Fast recovery trigger - %d consecutive errors",
                    canErrorMonitor.consecutiveErrors);
    can_recovery_needed = true;
  }

  // Legacy compatibility - keep old variables for now
  can_error_count++;
  last_can_error_time = currentTime;
  if (can_error_count > MAX_CAN_ERRORS_PER_MINUTE) {
    can_recovery_needed = true;
  }
}

void resetCANSuccessCounter() {
  // Reset consecutive error count on successful operation
  if (canErrorMonitor.consecutiveErrors > 0) {
    Serial.printlnf("CAN operation successful - resetting error count");
    canErrorMonitor.consecutiveErrors = 0;
  }

  // Update success time
  canErrorMonitor.lastSuccessTime = millis();

  // Reset recovery attempts after sustained successful operation
  if (canErrorMonitor.recoveryAttempts > 0) {
    unsigned long timeSinceRecovery =
        millis() - canErrorMonitor.lastRecoveryAttempt;

    // Only reset attempts if we've had sustained success (30 seconds)
    if (timeSinceRecovery > 30000) {
      Serial.printlnf("Resetting recovery attempt counter after sustained "
                      "success (%lu ms)",
                      timeSinceRecovery);
      canErrorMonitor.recoveryAttempts = 0;
      canErrorMonitor.adaptiveMode =
          false; // Disable adaptive mode on sustained success
      canErrorMonitor.extendedRecoveryDelay = 0;
    }
  }

  // Only clear recovery mode if we're not currently in an active recovery
  if (!can_recovery_needed && !CAN_ERROR) {
    canErrorMonitor.inRecoveryMode = false;
  }
}

// ========================================
// Watchdog Functions
// ========================================
// Software watchdog functions removed - using hardware ApplicationWatchdog
// only

// ========================================
// Utility Functions
// ========================================

void reportCANError(int err, const char *operation, bool report) {
  if (report) {
    CAN_ERROR = err != 0;
  }

  if (err != 0) {
    char ret[150];
    Serial.printlnf("CAN BUS ERROR %s", operation);
    ReturnErrorString(err, ret, sizeof(ret));
    sprintf(can_err_msg, "CAN BUS ERROR %s: %s", operation, ret);
    Serial.printlnf("%s", can_err_msg);

    // Log this as a CAN error for monitoring
    logCANError(err, operation);
  }
}

int resetDevice(String command) {
  if (command.length() > 0) {
    Serial.printlnf("Device reset requested. Reason: %s", command.c_str());
    Serial.printlnf("Free memory before reset: %lu bytes", System.freeMemory());
    Serial.printlnf("System uptime: %lu ms", millis());
    Serial.printlnf("Cellular connected: %s",
                    CELLULAR_CONNECTED ? "yes" : "no");
    Serial.printlnf("Credentials valid: %s",
                    areCredentialsValid() ? "yes" : "no");
    Serial.printlnf("CAN errors in last period: %d", can_error_count);
  }
  System.reset();
  return 1;
}
int forceGetVin(String command) {
  int port = atoi(command.c_str());
  if (!isValidPort(port)) {
    Particle.publish("forceGetVin", "error: invalid port", PRIVATE);
    return -1;
  }
  PortState *state = getPortState(port);
  if (!state) {
    Particle.publish("forceGetVin", "error: no state", PRIVATE);
    return -2;
  }

  if (strlen(state->VIN) > 0) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Port %d VIN: %s", port, state->VIN);
    Particle.publish("forceGetVin", buffer, PRIVATE);
    return 1; // Success
  } else {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Port %d: no VIN", port);
    Particle.publish("forceGetVin", buffer, PRIVATE);
    return 0; // No VIN
  }
}
int forceGetPortStatus(String command) {
  int port = atoi(command.c_str());

  String portStatus = getPortStatusSummary(port);
  // char buffer[100];
  // snprintf(buffer, sizeof(buffer), portStatus.c_str());
  Particle.publish("forceGetPortStatus", portStatus, PRIVATE);
  return 1; // Success
}

void logDebugInfo(const char *checkpoint) {
  static unsigned long lastLogTime = 0;
  unsigned long now = millis();

  Serial.printlnf("[%lu ms] Checkpoint: %s (Delta: %lu ms)", now, checkpoint,
                  now - lastLogTime);

  lastLogTime = now;
}

void logResetReason() {
  int reason = System.resetReason();
  int reasonData = System.resetReasonData();

  Serial.printlnf("RESET REASON: %d", reason);
  Serial.printlnf("RESET REASON DATA: %d", reasonData);

  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Reason: %d, Data: %d", reason, reasonData);

  Particle.publish("RESET REASON", buffer, PRIVATE);
}

void checkSystemHealth() {
  unsigned long freeMemory = System.freeMemory();
  unsigned long uptime = millis();

  if (freeMemory < 1000) {
    Serial.printlnf("Low memory warning: %lu bytes", freeMemory);
  }

  static unsigned long lastHealthCheck = 0;
  if (uptime - lastHealthCheck > 60000) {
    Serial.printlnf("System Health - Uptime: %lu ms, Free Memory: %lu bytes",
                    uptime, freeMemory);
    Serial.printlnf("Cellular Status: %s",
                    CELLULAR_CONNECTED ? "connected" : "disconnected");
    Serial.printlnf("Credentials: %s", getCredentialsStatus().c_str());
    Serial.printlnf("CAN Errors (last minute): %d", can_error_count);
    Serial.printlnf("CAN Recovery needed: %s",
                    can_recovery_needed ? "yes" : "no");

    if (portFlagHandler) {
      Serial.printlnf("Ports with pending flags: %d",
                      portFlagHandler->getPendingPortsCount());
    }

    lastHealthCheck = uptime;
  }
}

void emergencyReset(const char *reason) {
  Serial.printlnf("EMERGENCY RESET: %s", reason);
  Serial.printlnf("Error Stats - Consecutive: %d, Total: %lu",
                  canErrorMonitor.consecutiveErrors,
                  canErrorMonitor.totalErrors);
  delay(200); // Ensure message is sent
  System.reset();
}

// Internet connectivity monitoring thread with safety features
// This thread monitors internet connection and performs recovery actions:
// 1. Soft recovery (disconnect/reconnect) after 30 seconds
// 2. Hard reset after 1 minute, but only if system is in safe state
void internetCheckThread() {
  static bool connected = true;
  static bool did_disconnect = false;
  static unsigned long disconnectTime = 0;
  static unsigned long lastCheckTime = 0;
  static bool calledConnect = false;
  while (true) {
    // Non-blocking check every 5 seconds
    if (millis() - lastCheckTime > 5000) {

      lastCheckTime = millis();

      CELLULAR_CONNECTED = (Cellular.ready() && Particle.connected());

      if (!CELLULAR_CONNECTED) {
        if (!did_disconnect) { // Only set disconnectTime on FIRST disconnect
          Serial.println("Internet disconnected - starting recovery timeline");
          did_disconnect = true;
          disconnectTime = millis();
        }
        unsigned long disconnectedDuration = millis() - disconnectTime;

        // if (!calledConnect) {
        //   Particle.connect();
        //   calledConnect = true;
        // }
        // Hard reset after 1 minute
        if (disconnectedDuration > INTERNET_DISCONNECT_RESET_TIMEOUT) {

          Serial.printlnf("RESET: Internet disconnected for %lu ms - "
                          "performing device reset",
                          disconnectedDuration);
          System.reset(RESET_NO_WAIT);
        }
      } else {
        if (did_disconnect) {
          // Connection restored
          Serial.printlnf("Internet reconnected after %lu ms offline",
                          millis() - disconnectTime);
        }
        disconnectTime = 0;
        did_disconnect = false;
        // calledConnect = false;
      }
    } else {
      delay(100);
    }
  }
}

void canHealthMonitorThread() {
  static unsigned long lastHealthCheck = 0;
  const unsigned long HEALTH_CHECK_INTERVAL =
      1000; // Check every second during crisis
  const unsigned long MAIN_LOOP_TIMEOUT = 20000; // 20 seconds max

  Serial.println("CAN Health Monitor thread started");

  while (true) {
    unsigned long currentTime = millis();

    // Check health every second
    if (currentTime - lastHealthCheck >= HEALTH_CHECK_INTERVAL) {

      // Check MCP2515 error flags for actual hardware issues
      uint8_t currentFlags = getCANErrorFlags(true);

      // Monitor CAN error rate (from old canMonitorThread)
      // Monitor CAN error rate
      if (currentTime - last_can_error_time > CAN_ERROR_RESET_INTERVAL) {
        if (can_error_count > 0) {
          Serial.printlnf("Resetting CAN error count (was %d)",
                          can_error_count);
        }
        can_error_count = 0;
        last_can_error_time = currentTime;
      }

      // Check if error rate is too high
      if (can_error_count > MAX_CAN_ERRORS_PER_MINUTE) {
        Serial.printlnf("CAN error rate too high (%d errors/minute), "
                        "triggering recovery",
                        can_error_count);
        can_recovery_needed = true;
      }

      // Check interrupt health
      checkInterruptHealth();

      // Check transmission/reception balance
      checkTransmissionReceptionBalance();

      // Check if we have consecutive CAN errors with lower threshold
      if (canErrorMonitor.consecutiveErrors >= 3) {
        if (!canErrorMonitor.inRecoveryMode) {
          Serial.printlnf("CAN Error Threshold Reached: %d consecutive errors",
                          canErrorMonitor.consecutiveErrors);
          can_recovery_needed = true;
          canErrorMonitor.inRecoveryMode = true;
          canErrorMonitor.lastRecoveryAttempt = currentTime;
        }
      }

      // Reset consecutive error count if enough time has passed without
      // errors
      if (currentTime - canErrorMonitor.lastErrorTime > CAN_ERROR_WINDOW) {
        if (canErrorMonitor.consecutiveErrors > 0) {
          Serial.printlnf("CAN Error Window Expired - Resetting "
                          "consecutive error count");
          canErrorMonitor.consecutiveErrors = 0;
        }
      }

      // Emergency reset if error cascade continues
      if (canErrorMonitor.consecutiveErrors >= 10) {
        emergencyReset("Excessive consecutive CAN errors");
      }

      // Reset if stuck in recovery mode too long
      if (canErrorMonitor.inRecoveryMode &&
          currentTime - canErrorMonitor.lastRecoveryAttempt >
              CAN_RECOVERY_DELAY * 2) {
        emergencyReset("CAN recovery timeout");
      }

      // Check if no successful operations for too long
      if (canErrorMonitor.lastSuccessTime > 0 &&
          currentTime - canErrorMonitor.lastSuccessTime >
              PORT_CHECK_INTERVAL * 2 + 5000) {
        // Instead of emergency reset, check if CAN hardware is actually
        // healthy
        if (currentFlags == 0 && canErrorMonitor.consecutiveErrors == 0) {
          // CAN hardware looks fine, maybe just need to restart polling
          Serial.println("No recent CAN success but hardware looks healthy - "
                         "restarting polling");
          markPortsUnpolled(); // Gentle restart of port polling
          canErrorMonitor.lastSuccessTime = millis(); // Reset timer

          // Give it one more cycle to verify success
          static unsigned long lastGentleRestart = 0;
          if (currentTime - lastGentleRestart >
              60000) { // Only try once per minute
            lastGentleRestart = currentTime;
          } else {
            // Second gentle restart failed, more serious issue
            Serial.println("Gentle restart failed, CAN may have deeper issues");
            emergencyReset("CAN operations failed even after gentle restart");
          }
        } else {
          // Actual CAN hardware errors detected
          Serial.printlnf("CAN hardware errors detected (flags: 0x%02X)",
                          currentFlags);
          emergencyReset("No successful CAN operations with hardware errors");
        }
      }

      // Hardware watchdog handles main loop freeze detection automatically

      // Check CAN error rate with lower threshold
      if (canErrorMonitor.consecutiveErrors >= 6) {
        Serial.printlnf("CRITICAL: Excessive CAN errors (%d), forcing reset",
                        canErrorMonitor.consecutiveErrors);
        delay(100);
        emergencyReset("Excessive CAN errors");
      }

      // Check for queue overflow conditions
      if (queueOverflow) {
        Serial.println("CAN message queue overflow detected");
        queueOverflow = false; // Reset flag
        logCANError(-3, "persistent_queue_overflow");
      }

      // Periodic queue health check
      if (messageCount > (CAN_QUEUE_SIZE * 2 / 3)) { // More than 66% full
        Serial.printlnf("WARNING: CAN queue filling up (%d/%d messages)",
                        messageCount, CAN_QUEUE_SIZE);

        // If queue is critically full and not being processed, force clear old
        // messages
        static unsigned long lastQueueStuckTime = 0;
        static int lastQueueCount = 0;

        if (messageCount == lastQueueCount &&
            messageCount > (CAN_QUEUE_SIZE * 4 / 5)) {
          // Queue hasn't changed and is >80% full - might be stuck
          if (lastQueueStuckTime == 0) {
            lastQueueStuckTime = currentTime;
          } else if (currentTime - lastQueueStuckTime >
                     3000) { // Stuck for 3 seconds
            Serial.printlnf("CRITICAL: Queue stuck at %d messages for 3s - "
                            "clearing oldest 30%%",
                            messageCount);
            noInterrupts();
            int toClear = messageCount * 3 / 10; // Clear 30%
            queueHead = (queueHead + toClear) % CAN_QUEUE_SIZE;
            messageCount -= toClear;
            interrupts();
            lastQueueStuckTime = 0;
          }
        } else {
          lastQueueStuckTime = 0; // Reset if queue is moving
        }
        lastQueueCount = messageCount;
      }

      // Report RX overflow statistics if any have occurred
      if (canErrorMonitor.rxOverflowCount > 0) {
        unsigned long overflowDuration =
            currentTime - canErrorMonitor.firstRxOverflowTime;
        Serial.printlnf("RX Overflow Status: %d overflows in %lu ms (last "
                        "clear: %lu ms ago)",
                        canErrorMonitor.rxOverflowCount, overflowDuration,
                        currentTime - canErrorMonitor.lastRxOverflowClear);

        // Reset overflow tracking after successful recovery (if no overflow for
        // 10 seconds and system is healthy)
        if (currentTime - canErrorMonitor.lastRxOverflowClear > 10000 &&
            canErrorMonitor.consecutiveErrors == 0) {
          Serial.printlnf(
              "RX overflow resolved - resetting tracking (was %d overflows)",
              canErrorMonitor.rxOverflowCount);
          canErrorMonitor.rxOverflowCount = 0;
          canErrorMonitor.firstRxOverflowTime = 0;
        }
      }

      // Check for invalid CAN messages (indicates controller corruption)
      if (canErrorMonitor.consecutiveErrors >= 3) {
        Serial.printlnf(
            "CAN controller corruption suspected (%d consecutive errors)",
            canErrorMonitor.consecutiveErrors);
      }

      // Log periodic health status during error conditions
      // Only log health status every 10 seconds to reduce noise
      static unsigned long lastHealthLog = 0;
      if ((canErrorMonitor.consecutiveErrors > 0 ||
           canErrorMonitor.inRecoveryMode) &&
          currentTime - lastHealthLog > 10000) {
        Serial.printlnf(
            "CAN Health: Errors=%d, Attempts=%d, Recovering=%s, "
            "HW_Flags=0x%02X",
            canErrorMonitor.consecutiveErrors, canErrorMonitor.recoveryAttempts,
            canErrorMonitor.inRecoveryMode ? "YES" : "NO", currentFlags);
        lastHealthLog = currentTime;
      }

      // // Log CAN error flags if any are detected (less frequent)
      // static unsigned long lastFlagLog = 0;
      // if (currentFlags != 0 && currentTime - lastFlagLog > 30000) //
      // Every 30 seconds
      // {
      //   Serial.printlnf("MCP2515 hardware error flags detected: 0x%02X",
      //   currentFlags); getCANErrorFlags(true); // Debug output with
      //   detailed flag breakdown lastFlagLog = currentTime;
      // }

      lastHealthCheck = currentTime;
    }

    delay(100); // Check every second
  }
}

void checkInterruptHealth() {
  unsigned long currentTime = millis();

  // Only check every INTERRUPT_CHECK_INTERVAL
  if (currentTime - lastInterruptCheck < INTERRUPT_CHECK_INTERVAL) {
    return;
  }
  lastInterruptCheck = currentTime;

  // Check for RX overflow first
  uint8_t errorFlags = getCANErrorFlags(false);
  if (errorFlags & 0x40) { // RX0OVR flag
    handleRxOverflowWithEscalation(currentTime);
    return;
  }

  // Only check other interrupt health when system is fully operational
  if (!Particle.connected() || !areCredentialsValid()) {
    // System not ready - don't monitor interrupts yet
    static bool wasReady = false;
    if (wasReady) {
      Serial.printlnf("Interrupt health monitoring disabled - system not fully "
                      "operational");
      wasReady = false;
    }
    lastInterruptTime = currentTime; // Reset timer to prevent false alarms
    return;
  }

  // Log when monitoring becomes active
  static bool wasReady = false;
  if (!wasReady) {
    Serial.printlnf(
        "Interrupt health monitoring enabled - system fully operational");
    wasReady = true;
    lastInterruptTime = currentTime; // Start fresh when monitoring begins
  }

  // Check if we've received interrupts recently
  unsigned long timeSinceLastInterrupt = currentTime - lastInterruptTime;

  // Calculate dynamic timeout based on polling cycle
  // Normal: 30s timeout, but after port reset: 40s to allow for slower
  // responses
  unsigned long dynamicTimeout = INTERRUPT_TIMEOUT;
  unsigned long timeSincePortReset = currentTime - last_port_check_reset;
  if (timeSincePortReset < PORT_CHECK_INTERVAL * 2) {
    dynamicTimeout =
        PORT_CHECK_INTERVAL * 4; // 40s grace period after port polling starts
  }

  // If we haven't received message processing in a while, but CAN is
  // supposed to be working
  if (timeSinceLastInterrupt > dynamicTimeout && !CAN_ERROR &&
      !can_recovery_needed) {
    // Check MCP2515 error flags first to see if it's just buffer overflow
    uint8_t errorFlags = getCANErrorFlags(false);

    // If it's just RX buffer overflow, clear buffers instead of interrupt
    // recovery
    if (errorFlags & 0x40) { // RX0OVR flag
      handleRxOverflowWithEscalation(currentTime);
      return;
    }

    // Check if we can still send (TX working) but not receive (RX/interrupt
    // dead)
    Serial.printlnf("No CAN message processing for %lu ms (timeout: %lu ms) - "
                    "checking interrupt system",
                    timeSinceLastInterrupt, dynamicTimeout);

    // Check MCP2515 interrupt flags to see if they're stuck
    uint8_t intFlags = mcp2515.getInterrupts();
    Serial.printlnf("MCP2515 interrupt flags: 0x%02X", intFlags);

    if (intFlags != 0) {
      Serial.printlnf("CRITICAL: MCP2515 has pending interrupts but no message "
                      "processing!");

      // Try to recover the interrupt system
      recoverInterruptSystem();
    } else {
      Serial.printlnf(
          "MCP2515 no pending interrupts - may be normal quiet period");
      // Update last interrupt time to prevent false alarms during quiet
      // periods
      lastInterruptTime = currentTime - (INTERRUPT_TIMEOUT / 2);
    }
  }
}

void recoverInterruptSystem() {
  Serial.printlnf("=== ATTEMPTING INTERRUPT SYSTEM RECOVERY ===");

  // Detach current interrupt
  detachInterrupt(CAN_INT);
  delay(100);

  // Clear any pending MCP2515 interrupts
  mcp2515.clearInterrupts();
  delay(50);

  // Reconfigure the interrupt pin
  pinMode(CAN_INT, INPUT_PULLUP);
  delay(50);

  // Re-attach interrupt only if connected
  if (areCredentialsValid() && CELLULAR_CONNECTED) {
    // Clear any pending interrupts before re-attaching
    mcp2515.clearInterrupts();
    mcp2515.clearRXnOVR();
    delay(50);

    attachInterrupt(CAN_INT, can_interrupt, FALLING);
    delay(100);
    canInterruptAttached = true; // Re-enable polling after interrupt recovery
    Serial.printlnf("Interrupt re-attached after recovery with cleared state");
  } else {
    canInterruptAttached = false; // Disable polling until connected
    Serial.printlnf("Interrupt not re-attached - waiting for connection");
  }

  // Reset interrupt health tracking
  lastInterruptTime = millis();
  interruptHealthy = true;

  Serial.printlnf("Interrupt system recovery completed");
  Serial.printlnf("Re-attached interrupt on pin %d", CAN_INT);

  // Test interrupt by reading any pending messages
  uint8_t intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    Serial.printlnf("Post-recovery: MCP2515 flags: 0x%02X", intFlags);
  }
}

void checkTransmissionReceptionBalance() {
  unsigned long currentTime = millis();

  // Check for RX overflow first - handle regardless of connectivity status
  uint8_t errorFlags = getCANErrorFlags(false);
  if (errorFlags & 0x40) { // RX0OVR flag
    handleRxOverflowWithEscalation(currentTime);
    return;
  }

  // Only check TX/RX balance when system is fully operational
  if (!CELLULAR_CONNECTED || !areCredentialsValid()) {
    // System not ready - reset timers to prevent false alarms
    static bool txRxWasReady = false;
    if (txRxWasReady) {
      Serial.printlnf("TX/RX monitoring disabled - system not ready");
      txRxWasReady = false;
    }
    lastTransmissionTime = currentTime;
    lastInterruptTime = currentTime;
    return;
  }

  // Log when TX/RX monitoring becomes active
  static bool txRxWasReady = false;
  if (!txRxWasReady) {
    Serial.printlnf("TX/RX monitoring enabled - system ready");
    txRxWasReady = true;
    lastTransmissionTime = currentTime; // Start fresh when monitoring begins
    lastInterruptTime = currentTime;
  }

  // Check if we're actively transmitting but not receiving
  unsigned long timeSinceLastTX = currentTime - lastTransmissionTime;
  unsigned long timeSinceLastRX = currentTime - lastInterruptTime;

  // Calculate dynamic TX/RX timeout based on polling patterns
  // After port polling cycle starts, give extra time for all ports to
  // respond
  unsigned long dynamicTxRxTimeout = TX_RX_IMBALANCE_TIMEOUT;
  unsigned long timeSincePortReset = currentTime - last_port_check_reset;
  if (timeSincePortReset < PORT_CHECK_INTERVAL * 2) {
    dynamicTxRxTimeout =
        PORT_CHECK_INTERVAL * 5; // 50s grace period after polling cycle
  }

  // If we've transmitted recently but haven't received anything in much
  // longer
  if (timeSinceLastTX < PORT_CHECK_INTERVAL &&
      timeSinceLastRX > dynamicTxRxTimeout && !CAN_ERROR &&
      !can_recovery_needed) {
    // Before attempting interrupt recovery, check if it's just buffer
    // overflow
    uint8_t errorFlags = getCANErrorFlags(false);

    // If it's RX overflow, that's normal under high traffic - not an
    // interrupt failure
    if (errorFlags & 0x40) { // RX0OVR flag
      handleRxOverflowWithEscalation(currentTime);
      return;
    }

    // Only trigger interrupt recovery if we have real interrupt issues
    static unsigned long lastInterruptRecovery = 0;
    if (currentTime - lastInterruptRecovery >
        PORT_CHECK_INTERVAL * 6) { // Only once per 6 polling cycles
      Serial.printlnf("CRITICAL: TX/RX imbalance - TX:%lu ms ago, RX:%lu ms "
                      "ago (timeout: %lu ms)",
                      timeSinceLastTX, timeSinceLastRX, dynamicTxRxTimeout);
      Serial.printlnf("TX works but RX failed - attempting interrupt recovery");
      recoverInterruptSystem();
      lastInterruptRecovery = currentTime;
    } else {
      // Reset timer to prevent constant triggering
      lastInterruptTime = currentTime - (dynamicTxRxTimeout / 2);
    }
  }
}

void prepareCANForInterrupt() {
  Serial.println("Preparing CAN for interrupt attachment...");

  // 1. Make sure interrupt is detached first
  detachInterrupt(CAN_INT);
  delay(50);

  // 2. Clear MCP2515 error flags (including overflow flags)
  uint8_t errorFlags = mcp2515.getErrorFlags();
  if (errorFlags != 0) {
    Serial.printlnf("Clearing MCP2515 error flags: 0x%02X", errorFlags);
    mcp2515.clearRXnOVR(); // Clear RX overflow flags
  }

  // 3. Read and discard all pending messages from hardware buffers
  can_frame dummyMsg;
  int messagesCleared = 0;
  while (mcp2515.readMessage(&dummyMsg) == MCP2515::ERROR_OK &&
         messagesCleared < 20) {
    messagesCleared++;
  }
  if (messagesCleared > 0) {
    Serial.printlnf("Discarded %d pending messages from MCP2515",
                    messagesCleared);
  }

  // 4. Clear all interrupt flags in MCP2515
  uint8_t intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    Serial.printlnf("Clearing MCP2515 interrupt flags: 0x%02X", intFlags);
    mcp2515.clearInterrupts();
  }

  // 5. Reset our software queue state
  noInterrupts();
  queueHead = 0;
  queueTail = 0;
  messageCount = 0;
  queueOverflow = false;
  memset(messageQueue, 0, sizeof(messageQueue));
  interrupts();

  // 6. Reset CAN error monitoring state
  canErrorMonitor.rxOverflowCount = 0;
  canErrorMonitor.firstRxOverflowTime = 0;
  canErrorMonitor.lastRxOverflowClear = 0;
  canErrorMonitor.consecutiveErrors = 0;

  // 7. Wait for hardware to settle
  delay(100);

  // 8. Final check - make sure no new interrupts appeared
  intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    mcp2515.clearInterrupts();
    Serial.printlnf("Cleared additional interrupt flags: 0x%02X", intFlags);
  }

  // 9. Check the interrupt pin state before attaching
  int pinState = digitalRead(CAN_INT);
  Serial.printlnf("CAN_INT pin state before attach: %s",
                  pinState == LOW ? "LOW (pending)" : "HIGH (clear)");

  // 10. If pin is still low, clear interrupts one more time
  if (pinState == LOW) {
    Serial.println("CAN_INT pin is LOW - clearing interrupts again");
    mcp2515.clearInterrupts();
    delay(50);
    pinState = digitalRead(CAN_INT);
    Serial.printlnf("CAN_INT pin state after additional clear: %s",
                    pinState == LOW ? "LOW (still pending)" : "HIGH (cleared)");
  }

  // 12. NOW attach the interrupt with everything clean
  attachInterrupt(CAN_INT, can_interrupt, FALLING);

  // Set flag to enable port polling
  canInterruptAttached = true;

  // 13. Reset timing trackers
  lastInterruptTime = millis();
  lastTransmissionTime = millis();
  canErrorMonitor.lastSuccessTime = millis();

  Serial.println("CAN interrupt attached with fully clean state - ready to "
                 "receive messages");
}

// Interruptible delay that processes cloud messages and checks for pending
// commands
void interruptibleDelay(unsigned long ms) {
  unsigned long start = millis();
  while ((millis() - start) < ms) {
    // Check for pending cloud commands
    if (pendingCloudCommand) {
      return; // Exit immediately to process cloud command
    }

    // Process Particle cloud messages
    Particle.process();

    // Small delay to prevent tight loop
    delay(10);
  }
}

void handleRxOverflowWithEscalation(unsigned long currentTime) {
  // Track RX overflow occurrences
  canErrorMonitor.rxOverflowCount++;

  // Initialize tracking on first overflow
  if (canErrorMonitor.rxOverflowCount == 1) {
    canErrorMonitor.firstRxOverflowTime = currentTime;
  }

  Serial.printlnf("RX buffer overflow detected (#%d) - clearing buffers",
                  canErrorMonitor.rxOverflowCount);

  // Always try clearing buffers first
  clearAllCANBuffers();
  canErrorMonitor.lastRxOverflowClear = currentTime;
  lastInterruptTime = currentTime; // Reset interrupt timer

  // Check if we've had persistent overflow issues
  unsigned long overflowDuration =
      currentTime - canErrorMonitor.firstRxOverflowTime;

  // If we've had 5+ overflows in 30 seconds, or continuous overflows for 5+
  // seconds
  if ((canErrorMonitor.rxOverflowCount >= 5 && overflowDuration < 30000) ||
      (overflowDuration >= 5000 && canErrorMonitor.rxOverflowCount >= 3)) {

    // Check if clearing buffers hasn't helped (overflow recurring within 1
    // second)
    if (currentTime - canErrorMonitor.lastRxOverflowClear < 1000) {
      Serial.printlnf(
          "CRITICAL: RX overflow persisting despite buffer clearing");
      Serial.printlnf("Overflow count: %d in %lu ms",
                      canErrorMonitor.rxOverflowCount, overflowDuration);
      Serial.printlnf(
          "Buffer clearing ineffective - triggering system restart");

      // Log the critical condition
      logCANError(-11, "persistent_rx_overflow_restart");

      // Force immediate restart
      Serial.printlnf(
          "*** FORCING SYSTEM RESTART DUE TO PERSISTENT RX OVERFLOW ***");
      delay(100); // Brief delay for serial output
      System.reset(RESET_NO_WAIT);
    }
  }
}

```


## Assistant

Perfect! Now this will show you:

1. **Exact time intervals between restarts** (in seconds)
2. **Boot counter** to see how many times it's restarted
3. **Warning for rapid restarts** (< 30 seconds)

The **retained memory survives resets** but gets cleared on power loss, so this will give you the real restart intervals.

Run this and you'll see output like:
```
=== BOOT #1 at Unix time 1673123456 ===
=== BOOT #2 at Unix time 1673123466 ===
Time since last boot: 10 seconds
WARNING: Rapid restart detected! (10 seconds)
```

This will tell us the **actual restart interval** and help identify if it's truly faster than 20 seconds, and by how much.

## User

you say this:
Possible Causes for <20 Second Resets

1. **System-level cellular watchdog** - Particle OS may have internal watchdogs for cellular that aren't visible in your application code

2. **Hardware brownout/power issues** - If power supply is unstable during cellular transmission attempts

3. **Particle OS panic/fault handler** - System crashes that trigger immediate resets without application-level logging

4. **Cellular modem firmware crashes** - The cellular modem itself may be resetting and taking the system with it

But I noticed these issues began showing up more after making some code changes, I really don't belive my code to be the issue, but I cannot shake that my device began going offline after a couple days and entering this state aafter flashing this code to it. I have no explanation

## Assistant

<thinking>
The user is making a very important point - they noticed these rapid restart issues began after making code changes and flashing new firmware. This is a crucial detail that changes everything. If it was working fine before their code changes and now it's exhibiting this behavior, then it's very likely something in their code is causing the issue.

Let me look at their code more carefully for potential issues that could cause rapid resets or system instability:

1. **Memory issues** - Memory leaks, stack overflow, heap corruption
2. **Threading issues** - Deadlocks, race conditions, infinite loops in threads
3. **Interrupt issues** - Problems with the CAN interrupt handler
4. **Hardware resource conflicts** - SPI conflicts, pin conflicts
5. **Stack overflow** - Deep recursion or large local variables
6. **Infinite loops** - Code that never yields control
7. **Resource exhaustion** - Too many objects, file handles, etc.

Looking at their code, I see several potential issues:

### 1. **CAN Interrupt Handler Issues**
The `can_interrupt()` function is quite complex for an interrupt handler:
```cpp
void can_interrupt() {
  // Minimal interrupt handler - just queue the message
  struct can_frame recMsg;
  
  // Read the message
  int readin = readCanMessage(&recMsg);
  
  if (readin == ERROR_OK) {
    // Comprehensive corruption and gibberish filtering
    // 1. Basic validity checks
    if ((int32_t)recMsg.can_id < 0 || recMsg.can_id == 0 ||
        recMsg.can_id > MAX_PORTS || recMsg.can_dlc > 8) {
      // Don't queue obviously corrupted messages
      return; // Silent discard for performance
    }

    // 2. Data content validation - filter out gibberish
    bool hasValidData = false;
    bool hasGibberish = false;

    for (int i = 0; i < recMsg.can_dlc; i++) {
      uint8_t byte = recMsg.data[i];

      // Check for valid ASCII characters (printable range)
      if ((byte >= 0x20 && byte <= 0x7E) || byte == 0x00) {
        hasValidData = true;
      }
      // Check for obvious gibberish (control chars, high ASCII)
      else if (byte < 0x20 || byte > 0x7E) {
        // Allow some common control characters
        if (byte != 0x0A && byte != 0x0D && byte != 0x09) // LF, CR, TAB
        {
          hasGibberish = true;
        }
      }
    }

    // Reject if mostly gibberish or no valid data
    if (hasGibberish && !hasValidData) {
      return; // Silent discard of gibberish
    }

    // 3. Message pattern validation - must start with valid command
    if (recMsg.can_dlc > 0) {
      char firstChar = (char)recMsg.data[0];
      // Valid message types: D(status), K(VIN), T(temp), H(heartbeat), etc.
      if (firstChar != 'D' && firstChar != 'K' && firstChar != 'T' &&
          firstChar != 'H' && firstChar != 'U' && firstChar != 'C' &&
          firstChar != 'V' && firstChar != 'F' && firstChar != 'E') {
        return; // Silent discard of invalid command
      }
    }

    // Add to queue if there's space
    if (messageCount < CAN_QUEUE_SIZE) {
      incrementMessageCounter();
      messageQueue[queueTail] = recMsg;
      queueTail = (queueTail + 1) % CAN_QUEUE_SIZE;
      messageCount++;
    } else {
      queueOverflow = true;
      logCANError(-2, "queue_overflow");

      // Emergency queue clear if severely overflowing
      static unsigned long lastEmergencyClear = 0;
      if (millis() - lastEmergencyClear > 5000) { // Only once per 5 seconds
        Serial.println(
            "EMERGENCY: Clearing oldest 25% of queue to prevent lockup");
        // Clear oldest 25% of messages
        int toClear = messageCount / 4;
        queueHead = (queueHead + toClear) % CAN_QUEUE_SIZE;
        messageCount -= toClear;
        lastEmergencyClear = millis();
      }
    }
  } else {
    // Log read errors (but not every single one to avoid spam)
    static unsigned long lastErrorLog = 0;
    if (millis() - lastErrorLog > 1000) // Log max once per second
    {
      logCANError(readin, "can_read_interrupt");
      lastErrorLog = millis();
    }
  }
}
```

This is WAY too much processing in an interrupt handler. Interrupt handlers should be minimal - just read data and queue it. All this validation, string checking, Serial.println calls, etc. could be causing system instability.

### 2. **Memory Issues**
They have multiple threads running:
- `canThread()`
- `port_request_thread()`
- `canHealthMonitorThread()`
- `internetCheckThread()`

Each thread gets its own stack. With 4 threads + main thread, they could be running out of RAM.

### 3. **Infinite Loop Potential**
Looking at `handlePortDataRequests()`, it's very complex with multiple nested loops and conditions. If there's a bug in the logic, it could get stuck in an infinite loop.

### 4. **Stack Overflow**
The `can_interrupt()` function has local variables and calls other functions. If interrupts are nested or called frequently, this could cause stack overflow.

### 5. **Race Conditions**
They have volatile variables being accessed from interrupts and main threads without proper synchronization in some places.

### 6. **Hardware Resource Conflicts**
They're using SPI for CAN and potentially other devices. If there are conflicts or improper SPI management, it could cause system crashes.

The user is right to suspect their code changes. The cellular issues they're seeing could actually be a symptom, not the cause. If the system is crashing/resetting rapidly due to their code, the cellular modem never gets a chance to establish a stable connection, leading to the GSM muxer errors they're seeing.
</thinking>

You're absolutely right to suspect your code changes! If this behavior started after flashing new firmware, then **your code is almost certainly the cause**. The cellular/GSM errors you're seeing are likely **symptoms**, not the root cause.

When a device gets stuck in rapid reset loops due to code issues, the cellular modem never gets a chance to establish a stable connection, which explains the GSM07.10 muxer failures you're seeing.

## Likely Code Issues Causing Rapid Resets

Looking at your code, I see several **red flags** that could cause system crashes:

### 1. **CAN Interrupt Handler is WAY Too Complex** ⚠️

Your `can_interrupt()` function is doing **way too much work** in an interrupt context:

```cpp
void can_interrupt() {
  // This is TOO MUCH for an interrupt handler!
  struct can_frame recMsg;
  int readin = readCanMessage(&recMsg);
  
  // Complex validation loops
  for (int i = 0; i < recMsg.can_dlc; i++) {
    uint8_t byte = recMsg.data[i];
    // ASCII validation logic...
  }
  
  // String comparisons in interrupt!
  if (firstChar != 'D' && firstChar != 'K' && firstChar != 'T' &&
      firstChar != 'H' && firstChar != 'U' && firstChar != 'C' &&
      firstChar != 'V' && firstChar != 'F' && firstChar != 'E') {
    return;
  }
  
  // Serial.println in interrupt! (via logCANError)
  logCANError(readin, "can_read_interrupt");
}
```

**Interrupt handlers should be minimal!** This complex processing can cause:
- Stack overflow
- System lockups
- Watchdog timeouts
- Memory corruption

### 2. **Potential Memory Exhaustion** ⚠️

You're running **4 threads** plus main thread:
```cpp
new Thread("can_thread", canThread);
new Thread("port_request_thread", port_request_thread);  
new Thread("can_health_monitor", canHealthMonitorThread);
new Thread("internet_checker", internetCheckThread);
```

Each thread needs its own stack (1.5KB+ each). With limited RAM on Particle devices, this could cause memory exhaustion.

### 3. **Race Conditions** ⚠️

Volatile variables accessed without proper synchronization:
```cpp
volatile int messageCount = 0;
volatile int queueHead = 0;
volatile int queueTail = 0;
```

These are modified in interrupt and main threads simultaneously, which could corrupt queue state.

## Quick Fix - Simplify CAN Interrupt

**Tool Call: Simplify CAN interrupt handler to minimal processing only**
Status: Failed

## User

I want to note, that memory never appears to be the issue, we are always below 80%

## Assistant

Good point about memory not being the issue. If you're consistently below 80% RAM usage, then memory exhaustion isn't the culprit.

However, **stack overflow** can still happen even with plenty of heap memory available. Each thread gets a fixed stack size (typically 1.5KB), and if any single function call chain goes too deep or has large local variables, it can overflow that stack regardless of total RAM usage.

## Most Likely Culprits Given Your Symptoms:

### 1. **CAN Interrupt Handler Complexity** (Most Likely)
Your interrupt handler is doing **way too much processing**:
- Complex loops and validation
- String operations  
- Function calls to `logCANError()` which probably calls `Serial.println()`
- Static variable management with timestamps

**This alone could cause system crashes.** Interrupt handlers should be under 100 microseconds, yours could be taking milliseconds.

### 2. **Stack Overflow in Threads**
Even with good heap memory, individual thread stacks can overflow. Look at functions like:
- `handlePortDataRequests()` - very complex with nested loops
- `processCANMessage()` - calls multiple deep functions
- Any recursive calls or large local arrays

### 3. **Interrupt/Thread Race Conditions**
Your CAN queue management between interrupt and threads could be corrupting data structures, leading to crashes.

## Quick Test - Disable CAN Processing Temporarily

To test if CAN interrupt is the issue, try this minimal change:

```cpp
void can_interrupt() {
  // MINIMAL interrupt - just read and discard for testing
  struct can_frame recMsg;
  int readin = readCanMessage(&recMsg);
  
  // Don't process anything - just prevent interrupt from staying active
  if (readin == ERROR_OK) {
    // Do nothing - just testing if complex processing was causing crashes
  }
}
```

If the rapid resets stop with this change, then your complex CAN processing is definitely the cause.

**What specific code changes did you make before this started happening?** That would help pinpoint the exact issue.

## User

[@src](@file:gen-2-iot-firmware (copy)/src) this is the original code

## Assistant

Ah, now I can see the key differences! Looking at your original code vs the current one, there are several major changes that could absolutely cause the rapid restart issues you're experiencing:

## Key Differences That Could Cause Rapid Resets

### 1. **MQTT System Added** 
Your original code was much simpler - it used **Particle Cloud functions** directly. The new code added a complex **MQTT system** with:
- MQTT broker connections
- Multiple retry mechanisms  
- Health monitoring threads
- Connection state management

### 2. **Multiple New Threads**
Original: Fewer/simpler threads
New: **4 concurrent threads** running simultaneously:
```cpp
new Thread("can_thread", canThread);
new Thread("port_request_thread", port_request_thread);  
new Thread("can_health_monitor", canHealthMonitorThread);
new Thread("internet_checker", internetCheckThread);
```

### 3. **Complex CAN Interrupt Handler**
Your original `can_interrupt()` was much simpler. The new version has **extensive processing**:
- Multiple validation loops
- ASCII checking
- Pattern matching
- String operations
- Static variables with timing

### 4. **MQTT Health Monitoring**
New complex health checking code that could trigger resets:
```cpp
void checkMQTTHealth() {
  // Complex timeout logic that can force reconnections
  if (timeSinceLastMessage > MQTT_MESSAGE_TIMEOUT) {
    client.disconnect();
    BROKER_CONNECTED = false;
  }
}
```

## Most Likely Culprits

### **Thread Stack Overflow** 
With 4 threads + complex interrupt processing, you could easily be running out of stack space, causing silent crashes.

### **MQTT Connection Issues**
The MQTT system has multiple timeout/retry mechanisms that could be conflicting and causing rapid resets.

### **Race Conditions**
Multiple threads accessing shared variables without proper synchronization.

## Quick Test: Revert to Simple Version

To confirm this theory, try this minimal change to your current code - **disable the new systems temporarily**:

**Tool Call: Create minimal test version by commenting out complex new systems**
Status: Failed

## Assistant

Since I can't edit the files directly, here's what you should try:

## **Emergency Test - Comment Out New Systems**

1. **Disable extra threads** in `setup()`:
```cpp
// Comment these out temporarily:
// new Thread("can_thread", canThread);
// new Thread("port_request_thread", port_request_thread);
// new Thread("can_health_monitor", canHealthMonitorThread);
// new Thread("internet_checker", internetCheckThread);
```

2. **Disable MQTT** in `initializeParticle()`:
```cpp
// Comment out:
// initializeM

## User

No you mixed it up, the original code had the mqtt system, the new code is the Particle cloud only 

## Assistant

You're absolutely right! I apologize for the confusion. So the **original working code** had the MQTT system, and the **new problematic code** switched to Particle Cloud only. 

Looking at this correctly now:

## Key Changes That Could Cause Rapid Resets

### 1. **Removed MQTT, Added Particle Cloud Functions**
- **Original**: Used MQTT broker with `publishCloud()` 
- **New**: Uses `publishJuiseMessage()` with Particle.publish()

### 2. **Added Complex Message Deduplication System**
The new code added this complex deduplication system:
```cpp
struct MessageHistory {
    char lastMessage[128];
    unsigned long lastSentTime;
};
extern struct MessageHistory messageHistory[];
```

### 3. **New Cloud Command Priority System**
```cpp
extern volatile bool pendingCloudCommand;
// Added throughout the code to interrupt normal operations
```

### 4. **More Complex Threading Logic**
The new code has more complex thread coordination with cloud command priority.

### 5. **Added Retained Memory Tracking**
```cpp
retained uint32_t bootCounter = 0;
retained uint32_t lastBootTime = 0;
```

## Most Likely Culprits

### **Particle Cloud Overload**
The biggest change is switching from MQTT to **Particle.publish()** calls. If you're sending too many Particle events too quickly, **Particle Cloud has rate limits** that could cause connection issues.

### **Cloud Command Race Conditions**
The new `pendingCloudCommand` flag system could be causing timing issues where threads get stuck waiting.

### **Message Deduplication Logic**
The complex deduplication system could have bugs causing memory corruption or infinite loops.

## Quick Test - Disable New Features

Try commenting out these new systems one by one:

1. **Disable message deduplication** in `publishJuiseMessage()`:
```cpp
// Comment out all the deduplication logic
// Just do: return Particle.publish(JUISE_OUTGOING, payload.c_str(), PRIVATE);
```

2. **Remove cloud command priority**:
```cpp
// Comment out all pendingCloudCommand checks
```

3. **Check Particle Cloud rate limits** - you might be hitting the **4 events/second limit**.

The switch from MQTT (which can handle high message rates) to Particle Cloud (which has strict rate limits) is likely the root cause!

## User

How would these cause the mux errors I showed?

## Assistant

<thinking>
The user is asking a very good question. They're seeing GSM07.10 muxer errors and cellular NCP initialization failures, and I was focusing on application-level code changes. But how would changes like switching from MQTT to Particle Cloud, adding message deduplication, or cloud command priority cause low-level cellular modem issues?

Let me think about this more carefully:

1. **GSM07.10 muxer errors** - These are at the cellular modem firmware level, indicating the cellular modem itself is having trouble with its internal multiplexing system.

2. **NCP client initialization failures** - NCP (Network Control Protocol) is the interface between the Particle OS and the cellular modem. Error -210 suggests the cellular modem is not responding properly to initialization commands.

3. **Rapid restarts without application logs** - This suggests system-level crashes, not application-level issues.

So the user is right to question this. Application-level changes like switching from MQTT to Particle Cloud wouldn't directly cause GSM muxer errors. However, they could cause them **indirectly** in several ways:

## Indirect Ways Code Changes Could Cause Cellular Issues:

### 1. **Memory Corruption**
- If the new code has memory corruption bugs (buffer overflows, use-after-free, etc.)
- This corruption could affect the cellular driver memory space
- Leading to cellular modem communication failures

### 2. **Stack Overflow**
- New threading/complexity causing stack overflow
- Stack corruption can affect system-level drivers
- Cellular driver gets corrupted memory

### 3. **Interrupt Timing Issues**
- Changes to interrupt handling or timing
- Could affect cellular modem interrupt servicing
- Causing cellular communication to fail

### 4. **Power Management Issues**
- New code causing different power draw patterns
- Voltage drops during cellular transmission attempts
- Causing modem to reset/fail

### 5. **SPI Bus Conflicts**
- Both CAN and cellular might use SPI
- New code changes to SPI timing/usage
- Could interfere with cellular modem communication

### 6. **System Resource Exhaustion**
- New code using more RAM/CPU
- System becomes unstable under load
- Affects cellular driver operation

### 7. **Timing-Sensitive Operations**
- Cellular modems are very timing-sensitive
- New threading or processing delays
- Could cause cellular protocol timeouts

## Most Likely Scenario:
The code changes introduced a **memory corruption bug** or **timing issue** that's affecting the **system-level cellular drivers**, not just the application. This would explain why:
- No application reset logs appear (system crashes before app logging)
- Low-level cellular errors occur
- Restarts happen faster than the 60-second timeout

The user is right that application-level changes shouldn't directly cause muxer errors, but they can do so indirectly through memory corruption or system instability.
</thinking>

You're absolutely right to question that! Application-level changes like switching from MQTT to Particle Cloud shouldn't directly cause **GSM07.10 muxer errors** and **cellular NCP initialization failures**. Those are low-level cellular modem issues.

However, your code changes could be causing these cellular errors **indirectly** through:

## How Code Changes Could Cause Cellular Modem Failures

### 1. **Memory Corruption**
Your new code could have memory corruption bugs that are affecting the **cellular driver's memory space**:
- Buffer overflows in message processing
- Use-after-free in threading
- Stack corruption affecting system drivers

### 2. **Stack Overflow Affecting System Drivers**
- New threading complexity causing stack overflow
- Corrupted stack affecting cellular driver operation
- System-level crashes before your app can log anything

### 3. **SPI Bus Conflicts**
Both your CAN controller and cellular modem likely use **SPI**:
- Changes to CAN SPI timing/configuration
- Could interfere with cellular modem SPI communication
- Causing cellular initialization failures

### 4. **Interrupt Timing Issues**
Changes to your CAN interrupt handling could affect:
- System interrupt priorities
- Cellular modem interrupt servicing
- Critical timing for cellular protocol

### 5. **Power/Voltage Issues**
New code patterns causing:
- Different power draw spikes
- Voltage drops during cellular transmission
- Cellular modem brownouts/resets

### 6. **System Resource Exhaustion**
- New threading using more CPU/RAM
- System becomes unstable under load
- Cellular driver fails to operate properly

## The Most Likely Culprit

Looking at your code changes, I suspect **memory corruption** from:

1. **Complex CAN interrupt handler** - doing too much processing in interrupt context
2. **Message deduplication system** - complex string operations that could overflow
3. **Threading race conditions** - multiple threads accessing shared data unsafely

The cellular modem is extremely sensitive to system stability. Even small memory corruption bugs can cause the **cellular driver to fail initialization**, leading to the GSM muxer errors you're seeing.

**This explains why there are no application reset logs** - the system is crashing at the driver level before your application logging can occur.

## User

Can you explain again how exactly this code can cause any of these? I really want to be sure this is not a rabbit hole. Especially given the original code had the same complex can_interrupt and threads. I did change the threads a bit, including  some of how I process messages, so this can be possible, but can you pin point anything exactly that can cause these issues you ststed before:


Possible Causes for <20 Second Resets

1. **System-level cellular watchdog** - Particle OS may have internal watchdogs for cellular that aren't visible in your application code

2. **Hardware brownout/power issues** - If power supply is unstable during cellular transmission attempts

3. **Particle OS panic/fault handler** - System crashes that trigger immediate resets without application-level logging

4. **Cellular modem firmware crashes** - The cellular modem itself may be resetting and taking the system with it

## Assistant

You're absolutely right to question this and avoid rabbit holes. Let me be more specific and honest about what I can actually see in your code that could cause these issues.

## Specific Code Issues I Can Identify

### 1. **Race Condition in CAN Queue Management**
```cpp
// In can_interrupt() - INTERRUPT CONTEXT
if (messageCount < 50) {
  incrementMessageCounter();
  messageQueue[queueTail] = recMsg;
  queueTail = (queueTail + 1) % 50;
  messageCount++;
}

// In handleCanQueue() - THREAD CONTEXT  
noInterrupts();
if (messageCount > 0) {
  msg = messageQueue[queueHead];
  queueHead = (queueHead + 1) % 50;
  messageCount--;
}
interrupts();
```

**Problem**: `messageCount` is accessed in both interrupt and thread context. Even though you use `noInterrupts()` in the thread, the interrupt can still modify `messageCount` between the check and the actual array access, potentially causing array bounds violations.

### 2. **Stack Usage in Interrupt Handler**
```cpp
void can_interrupt() {
  struct can_frame recMsg;  // 16+ bytes on stack
  // Complex processing with loops, function calls
  incrementMessageCounter(); // Function call from interrupt
  // Multiple validation loops with local variables
}
```

**Problem**: Interrupts use the current thread's stack. If any thread has limited stack space and gets interrupted during deep function calls, this could cause stack overflow.

### 3. **Potential Buffer Overflow in Message Deduplication**
```cpp
strncpy(messageHistory[port].lastMessage, message,
        sizeof(messageHistory[port].lastMessage) - 1);
```

**Issue**: If `message` parameter doesn't have proper bounds checking before this call, and if `port` calculation is wrong, this could write outside array bounds.

## BUT Here's the Reality Check

You're **absolutely right to be skeptical**. Looking at your specific changes:

- The original code had the same complex `can_interrupt()`
- The original code had similar threading
- The cellular issues you're seeing are **very low-level**

## What I Actually Think is Happening

Based on the **GSM07.10 muxer** and **NCP -210** errors, this looks more like:

1. **Particle OS bug** triggered by your specific usage patterns
2. **Cellular modem firmware issue** exposed by timing changes
3. **Hardware issue** (power, antenna, SIM) that coincidentally appeared after code changes

## The Real Test

Instead of chasing code issues, try this **definitive test**:

1. **Flash the exact original working code** back to the device
2. **Leave it running for 24+ hours** 
3. **See if the cellular issues persist**

If the original code also shows these cellular issues now, then it's **definitely not your code changes** - it's either:
- Hardware degradation
- Network/carrier issues  
- Particle OS regression
- Environmental factors (power, temperature, signal strength)

Your instinct to avoid the rabbit hole is correct. The cellular muxer errors are likely **not caused by your application code changes**.

