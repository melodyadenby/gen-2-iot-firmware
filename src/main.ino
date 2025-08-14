#include "Arduino.h"
#include "DeviceInfoLedger.h"
#include "Particle.h"
#include "can.h"
#include "can_processor.h"
#include "config.h"
#include "credentials.h"
#include "fixes/json_compat.h"
#include "lights.h"
#include "logging.h"
#include "main.h"
#include "mqtt.h"
#include "port_event_handler.h"
#include "port_flag_handler.h"
#include "port_state.h"
#include "utils.h"
#include <ArduinoJson.h>
#include <MQTT.h>
#include <SPI.h>

// Product version setup
PRODUCT_VERSION(PRODUCT_VERSION_NUM);

// Global CAN state variables
char can_err_msg[200];
bool CAN_ERROR = false;

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

can_frame messageQueue[50];

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
void handleSerialCommands();

// Hardware watchdog handler
void hardwareWatchdogHandler() { System.reset(RESET_NO_WAIT); }

void setup() {
  initializeSystem();

  // Initialize hardware watchdog early - 20 second timeout
  hardwareWatchdog =
      new ApplicationWatchdog(20000, hardwareWatchdogHandler, 1536);
  Serial.printlnf("Hardware ApplicationWatchdog initialized (20s timeout)");

  initializeArchitecture();
  initializeHardware();

  if (!CAN_ERROR) {
    initializeParticle();
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
    resetDevice("CAN error during startup");
  }
}

void loop() {
  ApplicationWatchdog::checkin(); // Feed hardware watchdog
  DeviceInfoLedger::instance().loop();
  handleMQTTClientLoop();

  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();

  // Detect if main loop is running too slowly (potential freeze indicator)
  if (lastLoopTime > 0 && (currentTime - lastLoopTime) > 5000) {
    Serial.printlnf("WARNING: Main loop delay detected: %lu ms",
                    currentTime - lastLoopTime);
  }
  lastLoopTime = currentTime;
  handleSystemLoop();

  // Small delay to prevent CPU overload during error conditions
  if (CAN_ERROR || can_recovery_needed) {
    delay(10);
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

  // Initialize all subsystems
  initializePorts();
  initializeCredentials();
  initializeMQTT();
  initializeLedger();

  Particle.variable("CAN_ERROR", CAN_ERROR);
  Particle.variable("pub_id", MANUAL_MODE, STRING);
  Particle.variable("MQTT_connected", BROKER_CONNECTED);
  Particle.variable("credentialsFetched", credentialsFetched);
  Particle.variable("total_messages_received", total_messages_received);

  // VIN traffic monitoring variables
  Particle.variable("pending_vins", g_pendingVINRequests);
  Particle.variable("active_vins", g_activeVINRequests);
  Particle.variable("vin_flood_protection", g_vinFloodProtection);
  Particle.variable("last_vin_request", g_lastVINRequestTime);

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
  attachInterrupt(CAN_INT, can_interrupt, FALLING);

  // Initialize interrupt health tracking
  lastInterruptTime = millis();
  lastInterruptCheck = millis();
  interruptHealthy = true;

  // Start CAN processing thread
  new Thread("can_thread", canThread);
  new Thread("port_request_thread", port_request_thread);
  new Thread("can_health_monitor", canHealthMonitorThread);

  Serial.printlnf("Hardware initialized");
}

void initializeParticle() {
  // Register cloud functions
  Particle.function("resetDevice", resetDevice);

  // Connect to Particle Cloud
  Particle.connect();
  Particle.publishVitals();
  Particle.keepAlive(PARTICLE_KEEPALIVE_MIN * 60);

  // Enable system features
  System.enableFeature(FEATURE_RESET_INFO);
  System.enableFeature(FEATURE_RETAINED_MEMORY);

  // Wait for connection with timeout
  setLightRed();
  waitFor(Particle.connected, 60000);
  if (!Particle.connected()) {
    Serial.printlnf("Failed to connect to Particle Cloud");
    resetDevice("Cloud connection timeout");
    return;
  }

  // Log reset reason
  logResetReason();

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
    delay(100); // Prevent tight loop
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

  // Handle MQTT and credentials first
  handleMQTT();
  handleCredentials();
  updateSystemStatus();

  // System health monitoring
  checkSystemHealth();

  // Check for pending port status requests
  checkPortStatusRequest();

  // Handle serial debug commands
  handleSerialCommands();

  // Small delay to prevent CPU overload
  delay(5);
}

/**
 * Handle systematic port data requests with staggered polling
 * Polls each port in sequence with delays to avoid overwhelming the CAN bus
 */
void handlePortDataRequests() {
  // Add static variables for timing control
  static unsigned long last_function_run_time = 0;
  static bool first_run = true;
  static bool polling_cycle_active = false;
  static int current_poll_port = 1;

  // Only run this function on first call or once per PORT_CHECK_INTERVAL
  unsigned long current_time = millis();
  if (!first_run && !polling_cycle_active &&
      (current_time - last_function_run_time < PORT_CHECK_INTERVAL)) {
    // Too soon to run again, skip
    return;
  }

  // EMERGENCY STOP - Don't poll if we're in error cascade
  if (can_recovery_needed || CAN_ERROR) {
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
      delay(100);
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
    }

    // Start a new polling cycle
    if (!polling_cycle_active) {
      polling_cycle_active = true;
      current_poll_port = 1;
      Serial.println("Starting new polling cycle for all ports");
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

      Serial.printlnf("VIN Status: %d pending, %d active", pendingVINRequests,
                      activeVINRequests);
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
    if (current_time - last_poll_send_time < smart_delay) {
      // Not enough time has passed, skip this poll cycle
      return;
    }
  }

  // If we get here, we can poll
  Serial.println("TIME TO GET PORT DATA");

  // Check if portFlagHandler is available
  if (!portFlagHandler) {
    Serial.printlnf("portFlagHandler not initialized");
    return;
  }

  // Find next port that needs polling
  for (int attempts = 0; attempts < MAX_PORTS; attempts++) {
    if (current_poll_port > MAX_PORTS) {
      current_poll_port = 1; // Wrap around
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
      } else {
        Serial.printlnf("isVINRequest: %d", isVINRequest);
        Serial.printlnf("activeVINRequests: %d", activeVINRequests);
        Serial.printlnf("MAX_CONCURRENT_VINS: %d", MAX_CONCURRENT_VINS);
      }

      // This is the critical part - actually send the port data request
      Serial.printlnf("Attempting to poll port %d", current_poll_port);
      bool success = portFlagHandler->sendGetPortData(current_poll_port);

      if (success) {
        Serial.printlnf("Successfully polled port %d", current_poll_port);
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
          return; // Stop polling immediately
        }

        // Hardware watchdog will handle timeout protection automatically
      }

      last_poll_send_time = current_time;
      current_poll_port++; // Move to next port for next iteration

      // Instead of breaking, return to allow delay between ports
      return;
    } else {
      current_poll_port++;
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
  if (areCredentialsValid() && isMQTTConnected()) {
    setLightGreen(); // All systems operational

    // Set the just connected flag when we first get valid credentials and MQTT
    // connection
    static bool wasConnectedBefore = false;
    if (!wasConnectedBefore) {
      justConnectedFlag = true;
      wasConnectedBefore = true;
      Serial.printlnf("First successful connection detected - immediate port "
                      "polling enabled");
    }
  } else if (areCredentialsValid()) {
    setLightPurple(); // Have credentials, connecting to MQTT
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
    if (areCredentialsValid() && isMQTTConnected() && Particle.connected() &&
        !CAN_ERROR && !can_recovery_needed) {
      // Process incoming CAN messages
      handleCanQueue();

      // Process port flags (replaces old flagHandler)
      if (portFlagHandler) {
        portFlagHandler->processAllPortFlags();
      }

      // Small delay to prevent busy-waiting
      delay(10);
    } else {
      // Small delay to prevent busy-waiting
      delay(10);
    }

    // Feed hardware watchdog during port request thread
    ApplicationWatchdog::checkin();
  }
}

void port_request_thread() {
  while (true) {
    if (areCredentialsValid() && isMQTTConnected() && Particle.connected() &&
        !CAN_ERROR && !can_recovery_needed) {
      handlePortDataRequests();

      // Shorter delay to allow staggered polling to work properly
      // Each port will be polled with the appropriate delay between them
      delay(50);
    } else {
      // Small delay to prevent busy-waiting
      delay(100);
    }

    // Feed hardware watchdog during CAN thread
    ApplicationWatchdog::checkin();
  }
}

void handleCanQueue() {
  int messagesProcessed = 0;
  const int MAX_MESSAGES_PER_LOOP = 8;

  while (messageCount > 0 && messagesProcessed < MAX_MESSAGES_PER_LOOP) {
    can_frame msg;
    bool validMessage = false;

    // Thread-safe message extraction
    noInterrupts();
    if (messageCount > 0) {
      if (queueHead >= 0 && queueHead < 50 && queueTail >= 0 &&
          queueTail < 50) {
        msg = messageQueue[queueHead];
        queueHead = (queueHead + 1) % 50;
        messageCount--;
        validMessage = true;

        // Handle queue overflow reset
        if (queueOverflow && messageCount < 25) {
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
    Serial.printlnf("Received CAN message %s", recMsg.data);
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
    if (messageCount < 50) {
      incrementMessageCounter();
      messageQueue[queueTail] = recMsg;
      queueTail = (queueTail + 1) % 50;
      messageCount++;
    } else {
      queueOverflow = true;
      logCANError(-2, "queue_overflow");
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
  can_recovery_needed = false; // Clear the flag first

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
  attachInterrupt(CAN_INT, can_interrupt, FALLING);
  Serial.printlnf("CAN interrupt re-enabled on pin %d", CAN_INT);

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
    Serial.printlnf("MQTT connected: %s", isMQTTConnected() ? "yes" : "no");
    Serial.printlnf("Credentials valid: %s",
                    areCredentialsValid() ? "yes" : "no");
    Serial.printlnf("CAN errors in last period: %d", can_error_count);

    if (isMQTTConnected()) {
      Serial.printlnf("Closing MQTT connection...");
    }

    delay(1000);
  }

  System.reset();
  return 1;
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
    Serial.printlnf("MQTT Status: %s", getMQTTStatus().c_str());
    Serial.printlnf("MQTT Healthy: %s", isMQTTHealthy() ? "yes" : "no");
    Serial.printlnf("MQTT Fail Count: %d", getMQTTFailCount());
    Serial.printlnf("Credentials: %s", getCredentialsStatus().c_str());
    Serial.printlnf("CAN Errors (last minute): %d", can_error_count);
    Serial.printlnf("CAN Recovery needed: %s",
                    can_recovery_needed ? "yes" : "no");

    // Force MQTT reconnection if unhealthy
    if (BROKER_CONNECTED && !isMQTTHealthy()) {
      Serial.println("MQTT unhealthy despite connection - forcing reconnect");
      forceMQTTReconnect();
    }

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
        Serial.printlnf(
            "CAN error rate too high (%d errors/minute), triggering recovery",
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
          Serial.printlnf(
              "CAN Error Window Expired - Resetting consecutive error count");
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

      // Report RX overflow statistics if any have occurred
      if (canErrorMonitor.rxOverflowCount > 0) {
        unsigned long overflowDuration =
            currentTime - canErrorMonitor.firstRxOverflowTime;
        Serial.printlnf("RX Overflow Status: %d overflows in %lu ms (last "
                        "clear: %lu ms ago)",
                        canErrorMonitor.rxOverflowCount, overflowDuration,
                        currentTime - canErrorMonitor.lastRxOverflowClear);
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
      // if (currentFlags != 0 && currentTime - lastFlagLog > 30000) // Every
      // 30 seconds
      // {
      //   Serial.printlnf("MCP2515 hardware error flags detected: 0x%02X",
      //   currentFlags); getCANErrorFlags(true); // Debug output with
      //   detailed flag breakdown lastFlagLog = currentTime;
      // }

      lastHealthCheck = currentTime;
    }

    // Feed hardware watchdog during health monitoring
    ApplicationWatchdog::checkin();

    delay(1000); // Check every second
  }
}

void checkInterruptHealth() {
  unsigned long currentTime = millis();

  // Only check every INTERRUPT_CHECK_INTERVAL
  if (currentTime - lastInterruptCheck < INTERRUPT_CHECK_INTERVAL) {
    return;
  }
  lastInterruptCheck = currentTime;

  // Only check interrupt health when system is fully operational
  if (!Particle.connected() || !BROKER_CONNECTED || !areCredentialsValid()) {
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

  // If we haven't received message processing in a while, but CAN is supposed
  // to be working
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

  // Re-attach interrupt
  attachInterrupt(CAN_INT, can_interrupt, FALLING);
  delay(100);

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

  // Only check TX/RX balance when system is fully operational
  if (!Particle.connected() || !BROKER_CONNECTED || !areCredentialsValid()) {
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
  // After port polling cycle starts, give extra time for all ports to respond
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

  // Reset overflow tracking after successful clear (if no overflow for 10
  // seconds)
  if (canErrorMonitor.rxOverflowCount > 0 &&
      currentTime - canErrorMonitor.lastRxOverflowClear > 10000) {
    Serial.printlnf(
        "RX overflow resolved - resetting tracking (was %d overflows)",
        canErrorMonitor.rxOverflowCount);
    canErrorMonitor.rxOverflowCount = 0;
    canErrorMonitor.firstRxOverflowTime = 0;
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "MQTT_STATUS") {
      Serial.printlnf("=== MQTT Status ===");
      Serial.printlnf("Connected: %s", isMQTTConnected() ? "yes" : "no");
      Serial.printlnf("Healthy: %s", isMQTTHealthy() ? "yes" : "no");
      Serial.printlnf("Status: %s", getMQTTStatus().c_str());
      Serial.printlnf("Fail Count: %d", getMQTTFailCount());
      Serial.printlnf("Last Send: %lu ms ago", millis() - getLastMQTTSend());
      Serial.printlnf(
          "Last Message Received: %lu ms ago",
          lastMqttMessageReceived > 0 ? millis() - lastMqttMessageReceived : 0);
      Serial.printlnf("Pub Topic: %s", MQTT_PUB_TOPIC);
      Serial.printlnf("Sub Topic: %s", MQTT_SUB_TOPIC);
    } else if (command == "MQTT_RECONNECT") {
      Serial.println("Forcing MQTT reconnection...");
      forceMQTTReconnect();
    } else if (command == "MQTT_HEARTBEAT") {
      Serial.println("Sending MQTT heartbeat...");
      publishCloud("H,0,1");
    } else if (command == "SYSTEM_STATUS") {
      Serial.printlnf("=== System Status ===");
      Serial.printlnf("Free Memory: %lu bytes", System.freeMemory());
      Serial.printlnf("Uptime: %lu ms", millis());
      Serial.printlnf("Particle Connected: %s",
                      Particle.connected() ? "yes" : "no");
      Serial.printlnf("Credentials Valid: %s",
                      areCredentialsValid() ? "yes" : "no");
      Serial.printlnf("CAN Error Count: %d", can_error_count);
      Serial.printlnf("CAN Recovery Needed: %s",
                      can_recovery_needed ? "yes" : "no");
    } else if (command == "HELP") {
      Serial.println("=== Available Commands ===");
      Serial.println("MQTT_STATUS - Show MQTT connection details");
      Serial.println("MQTT_RECONNECT - Force MQTT reconnection");
      Serial.println("MQTT_HEARTBEAT - Send test heartbeat");
      Serial.println("SYSTEM_STATUS - Show system health");
      Serial.println("HELP - Show this help");
    } else if (command.length() > 0) {
      Serial.printlnf("Unknown command: %s (type HELP for available commands)",
                      command.c_str());
    }
  }
}
