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
volatile bool CAN_INTERRUPT_FLAG = false;

// Architecture components
PortEventHandler *portEventHandler = nullptr;
PortFlagHandler *portFlagHandler = nullptr;

volatile bool queueOverflow = false;
volatile int messageCount = 0;
volatile int queueHead = 0;
volatile int queueTail = 0;

// Secondary processing queue (for thread-safe message handling)
#define PROCESSING_QUEUE_SIZE 50
can_frame processingQueue[PROCESSING_QUEUE_SIZE];
int processingQueueHead = 0;
int processingQueueTail = 0;
int processingQueueCount = 0;
os_mutex_t processingQueueMutex;

// Queue health telemetry
volatile uint32_t totalMessagesDropped = 0;
volatile uint32_t totalMessagesProcessed = 0;
volatile uint32_t maxQueueDepth = 0;

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

struct OutOfMemoryTracker {
  bool wasOutOfMemory;
  int outOfMemorySize;
  int freeMemoryAtRestart;
  unsigned long uptimeAtOOM;
} oomTracker = {false, 0, 0, 0};

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

// Size of memory when out of memory
int outOfMemory = -1;

// Function declarations
void checkInterruptHealth();
void recoverInterruptSystem();
void checkTransmissionReceptionBalance();
void handleRxOverflowWithEscalation(unsigned long currentTime);
void prepareCANForInterrupt();
void interruptibleDelay(unsigned long ms);
void outOfMemoryHandler(system_event_t event, int param);
void resetCANSuccessCounter();
void handleCANInterrupt();

// Hardware watchdog handler
void hardwareWatchdogHandler() {
  System.reset(RESET_REASON_WATCHDOG, RESET_NO_WAIT);
}

void setup() {
  // Enable out of memory handler for automatic memory monitoring
  System.on(out_of_memory, outOfMemoryHandler);

  initializeSystem();

  // Log startup time and reset reason IMMEDIATELY
  Log.info("=== DEVICE STARTUP AT %lu ms ===", millis());
  Log.info("System version: %s", System.version().c_str());
  Log.info("Free memory: %lu bytes", System.freeMemory());

  // Check for previous out of memory condition
  if (oomTracker.wasOutOfMemory) {
    Log.info("=== OUT OF MEMORY EVENT DETECTED ===");
    Log.info("Previous restart was due to out of memory");
    Log.info("Allocation size that failed: %lu bytes",
             oomTracker.outOfMemorySize);
    Log.info("Free memory at restart: %lu bytes",
             oomTracker.freeMemoryAtRestart);
    Log.info("Uptime when OOM occurred: %lu ms", oomTracker.uptimeAtOOM);
    Log.info("Current free memory: %lu bytes", System.freeMemory());
    // Clear the flag for next boot
    oomTracker.wasOutOfMemory = false;
  }

  initializeArchitecture();
  initializeHardware();

  if (!CAN_ERROR) {
    initializeParticle();

    // Initialize hardware watchdog - 20 second timeout
    hardwareWatchdog =
        new ApplicationWatchdog(20000, hardwareWatchdogHandler, 1536);
    Log.info("Hardware ApplicationWatchdog initialized (20s timeout)");

    // Initialize processing queue mutex
    os_mutex_create(&processingQueueMutex);

    // Start alternate threads
    new Thread("can_thread", canThread);
    new Thread("port_request_thread", port_request_thread);
    new Thread("can_health_monitor", canHealthMonitorThread);
    new Thread("internet_checker", internetCheckThread);

    requestCredentials();

    // Initialize recovery state
    canErrorMonitor.lastSuccessTime = millis();
    canErrorMonitor.recoveryAttempts = 0;

    Log.info("=== SYSTEM STARTUP COMPLETE ===");
    Log.info("CAN Error Monitoring: ENABLED");
    Log.info("Hardware Watchdog: ENABLED (20s timeout)");
    Log.info("Recovery System: READY");
  } else {
    Log.warn("=== STARTUP FAILED - CAN ERROR DETECTED ===");
    Log.warn("Uptime before CAN error reset: %lu ms", millis());
    resetDevice("CAN error during startup");
  }
}

void loop() {
  ApplicationWatchdog::checkin(); // Feed hardware watchdog
  DeviceInfoLedger::instance().loop();

  if (CAN_INTERRUPT_FLAG)
    handleCANInterrupt();

  // Check system health first before any operations
  checkSystemHealth();

  // Process cloud messages FIRST to receive any pending commands
  Particle.process();

  // Check if cloud command already pending
  if (pendingCloudCommand) {
    Log.info("Cloud command pending - priority processing");
    return; // Skip everything else this iteration
  }

  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();

  // Detect if main loop is running too slowly (potential freeze indicator)
  if (lastLoopTime > 0 && (currentTime - lastLoopTime) > 5000) {
    Log.info("WARNING: Main loop delay detected: %lu ms (uptime: %lu ms)",
             currentTime - lastLoopTime, currentTime);
  }
  lastLoopTime = currentTime;

  // Process CAN messages from the processing queue (lightweight)
  if (!CAN_ERROR && !can_recovery_needed) {
    // Check before processing
    Particle.process();
    if (pendingCloudCommand)
      return;

    processMessagesFromQueue();

    // Check after processing
    Particle.process();
    if (pendingCloudCommand)
      return;
  }

  // Process port flags in main loop (safer than in thread)
  if (portFlagHandler && areCredentialsValid() && CELLULAR_CONNECTED &&
      !CAN_ERROR && !can_recovery_needed) {
    // Check before flag processing
    Particle.process();
    if (pendingCloudCommand)
      return;

    portFlagHandler->processAllPortFlags();

    // Check after flag processing
    Particle.process();
    if (pendingCloudCommand)
      return;
  }

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

  Log.info("Architecture components initialized");
}

void initializeSystem() {

  // delay(2000);

  // Track boot times using retained memory
  bootCounter++;
  uint32_t currentTime = Time.now();
  Log.info("=== BOOT #%lu at Unix time %lu ===", bootCounter, currentTime);

  if (bootCounter > 1 && lastBootTime > 0) {
    uint32_t timeBetweenBoots = currentTime - lastBootTime;
    Log.info("Time since last boot: %lu seconds", timeBetweenBoots);
    if (timeBetweenBoots < 30) {
      Log.info("WARNING: Rapid restart detected! (%lu seconds)",
               timeBetweenBoots);
    }
  }
  lastBootTime = currentTime;

  // Initialize all subsystems
  initializePorts();
  initializeCredentials();
  initializeLedger();

  Log.info("*** KUHMUTE IoT V %s ***", BUILD_VERSION);
  Log.info("Device ID: %s", Particle.deviceID().c_str());
  Log.info("Environment: %s", getCurrentEnvironment());
  Log.info("System initialized");
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

  // CRITICAL: Clear ALL buffers BEFORE entering normal mode
  // This prevents startup overflow when all ports are docked and sending VINs
  Log.info(
      "Pre-startup: Clearing any existing CAN messages before normal mode...");

  can_frame dummyMsg;
  int preStartupCleared = 0;

  // Clear messages that might be in buffers from previous operations
  while (mcp2515.readMessage(&dummyMsg) == MCP2515::ERROR_OK &&
         preStartupCleared < 100) {
    preStartupCleared++;
  }

  // Clear any error flags
  mcp2515.clearRXnOVR();
  mcp2515.clearInterrupts();

  if (preStartupCleared > 0) {
    Log.info("Pre-startup cleared %d existing messages", preStartupCleared);
  }

  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK) {
    reportCANError(err, "setNormalMode", true);
    return;
  }

  Log.info("CAN controller: Normal mode OK!");

  // Extended startup drain to prevent overflow during vulnerable period
  Log.info("Starting extended startup drain...");
  unsigned long drainStart = millis();
  int totalDrained = 0;

  // Drain for 5 seconds to cover entire startup sequence
  while (millis() - drainStart < 5000) {
    int drainedThisLoop = 0;

    // Drain all available messages
    while (mcp2515.readMessage(&dummyMsg) == MCP2515::ERROR_OK &&
           drainedThisLoop < 50) {
      totalDrained++;
      drainedThisLoop++;
    }

    // Clear overflow flags if they occur
    uint8_t flags = mcp2515.getErrorFlags();
    if (flags & 0xC0) { // RX0OVR or RX1OVR
      mcp2515.clearRXnOVR();
      Log.info("Prevented overflow during drain (total: %d)", totalDrained);
    }

    delay(50); // 50ms between cycles
  }

  Log.info("Extended drain complete: cleared %d messages over 5 seconds",
           totalDrained);

  pinMode(CAN_INT, INPUT_PULLUP);
  // NOTE: CAN interrupt will be attached after connection is established
  // This prevents accumulating unwanted messages during startup

  // Initialize interrupt health tracking
  lastInterruptTime = millis();
  lastInterruptCheck = millis();
  interruptHealthy = true;

  Log.info("Hardware initialized (CAN interrupt pending)");
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
    Log.info("Failed to connect to Particle Cloud");
    resetDevice("");
  }

  // Log reset reason
  logResetReason();

  setLightBlue();
  Log.info("Particle Cloud connected");
}

void handleSystemLoop() {
  // Handle CAN recovery IMMEDIATELY - before anything else
  if (can_recovery_needed) {
    Log.info("EMERGENCY: Executing CAN recovery due to error cascade");
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
    Log.info("Failed to fetch credentials after max attempts");
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
      Log.info("Re-enabling port polling - errors have cleared (waited %lu ms)",
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
      Log.info("🚨 DID_PORT_CHECK reset for all ports 🚨");

      // Force start new cycle when ports are reset
      polling_cycle_active = false;
      current_poll_port = 0; // Will be set to 1 when cycle actually starts
    }

    // Start a new polling cycle
    if (!polling_cycle_active) {
      polling_cycle_active = true;
      current_poll_port = 1; // NOW set to port 1 when actually starting
      Log.info("Starting new polling cycle for all ports - starting at port %d",
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
    Log.info("portFlagHandler not initialized");
    return;
  }

  // Process cloud messages to check for incoming commands
  Particle.process();

  // Only process port polling when we're in an active cycle
  if (!polling_cycle_active) {
    return; // Don't increment port counter between cycles
  }

  // Safety check - if current_poll_port is 0, set it to 1
  if (current_poll_port == 0) {
    current_poll_port = 1;
    Log.info("WARNING: current_poll_port was 0, resetting to 1");
  }

  // Find next port that needs polling
  int start_port = current_poll_port; // Remember where we started
  for (int attempts = 0; attempts < MAX_PORTS; attempts++) {
    // Check for cloud commands every few ports
    if (attempts > 0 && attempts % 3 == 0) {
      Particle.process();
      if (pendingCloudCommand) {
        // Don't disable polling_cycle_active - just exit to handle command
        return; // Will resume polling in next iteration
      }
    }
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
        Log.info("PREVENTIVE: Skipping VIN request for port %d - %d "
                 "already active (max %d)",
                 current_poll_port, activeVINRequests, MAX_CONCURRENT_VINS);
        current_poll_port++; // Move to next port
        continue;            // Skip this VIN request
      }

      // This is the critical part - actually send the port data request
      bool success = portFlagHandler->sendGetPortData(current_poll_port);

      if (success) {
        Log.info("Polled port %d", current_poll_port);
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

          Log.info("VIN request sent to port %d - slot reserved, %d "
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
        Log.info("Failed to poll port %d (failures: %d)", current_poll_port,
                 portFailureCount[current_poll_port]);

        // Skip this port if it's failing repeatedly
        if (portFailureCount[current_poll_port] >= 3) {
          Log.info("Port %d has failed %d times - skipping for this cycle",
                   current_poll_port, portFailureCount[current_poll_port]);
          markPortPolled(current_poll_port); // Mark as "polled" to skip it
          portFailureCount[current_poll_port] = 0; // Reset for next cycle
        }

        logCANError(-1, "port_polling");

        // If too many consecutive errors, trigger IMMEDIATE recovery
        if (canErrorMonitor.consecutiveErrors >= 3) {
          Log.info("CRITICAL: Immediate CAN recovery needed");
          can_recovery_needed = true;
          pollingDisabled = true;
          pollingDisabledTime = millis();
          pendingCloudCommand = false; // Clear flag during error
          return;                      // Stop polling immediately
        }

        // Hardware watchdog will handle timeout protection automatically
      }

      last_poll_send_time = current_time;

      // Check for cloud messages after each port poll
      Particle.process();

      // Exit immediately if cloud command arrived
      if (pendingCloudCommand) {
        // Don't disable polling - just exit to handle command
        return; // Will resume from current_poll_port next time
      }

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

    // Check for cloud messages even when skipping ports
    if (attempts % 4 == 0) { // Every 4 ports checked
      Particle.process();
      if (pendingCloudCommand) {
        // Keep polling active, just exit to process command
        return; // Will continue from where we left off
      }
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
    Log.info(
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
      Log.info("First successful connection detected - immediate port "
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
  Log.info("CAN thread started - lightweight message transfer only");

  while (true) {
    // Simple job: move messages from interrupt queue to processing queue
    // No heavy processing, no cloud operations, no flag handling

    if (messageCount > 0) {
      transferMessagesToProcessingQueue();
    }

    // Small delay to prevent busy-waiting
    delay(5); // Shorter delay since we're doing less work
  }
}

// Lightweight function to transfer messages from interrupt queue to processing
// queue
void transferMessagesToProcessingQueue() {
  int transferred = 0;
  const int MAX_TRANSFER_PER_LOOP = 10;

  while (messageCount > 0 && transferred < MAX_TRANSFER_PER_LOOP) {
    can_frame msg;
    bool gotMessage = false;

    // Get message from interrupt queue (minimal time with interrupts disabled)
    noInterrupts();
    if (messageCount > 0 && queueHead >= 0 && queueHead < CAN_QUEUE_SIZE) {
      msg = messageQueue[queueHead];
      queueHead = (queueHead + 1) % CAN_QUEUE_SIZE;
      messageCount--;
      gotMessage = true;

      // Clear overflow flag when queue is manageable
      if (queueOverflow && messageCount < (CAN_QUEUE_SIZE / 2)) {
        queueOverflow = false;
      }
    }
    interrupts();

    // Add to processing queue (with mutex for thread safety)
    if (gotMessage) {
      os_mutex_lock(processingQueueMutex);
      if (processingQueueCount < PROCESSING_QUEUE_SIZE) {
        processingQueue[processingQueueTail] = msg;
        processingQueueTail = (processingQueueTail + 1) % PROCESSING_QUEUE_SIZE;
        processingQueueCount++;
        transferred++;
      } else {
        Log.info("WARNING: Processing queue full!");
      }
      os_mutex_unlock(processingQueueMutex);
    }
  }
}

// Process messages from the secondary queue (called from main loop)
void processMessagesFromQueue() {
  int processed = 0;
  const int MAX_PROCESS_PER_LOOP = 5; // Process fewer at a time in main loop

  while (processingQueueCount > 0 && processed < MAX_PROCESS_PER_LOOP) {
    // Check for cloud messages every 2 CAN messages
    if (processed > 0 && processed % 2 == 0) {
      Particle.process();
      if (pendingCloudCommand)
        return; // Exit immediately
    }

    can_frame msg;
    bool gotMessage = false;

    // Get message from processing queue
    os_mutex_lock(processingQueueMutex);
    if (processingQueueCount > 0) {
      msg = processingQueue[processingQueueHead];
      processingQueueHead = (processingQueueHead + 1) % PROCESSING_QUEUE_SIZE;
      processingQueueCount--;
      gotMessage = true;
    }
    os_mutex_unlock(processingQueueMutex);

    // Process the message (heavy work done in main loop context)
    if (gotMessage) {
      processCANMessage(msg);
      totalMessagesProcessed++;
      processed++;

      // Check for cloud messages every few CAN messages
      if (processed % 2 == 0) {
        Particle.process();
      }
    }
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

void processCANMessage(const can_frame &rawMessage) {
  // Check for corrupted CAN ID (negative or excessively large values)
  if ((int32_t)rawMessage.can_id < 0 || rawMessage.can_id > 16777215) {
    Log.error("CRITICAL: Corrupted CAN ID detected: %ld (0x%lX)",
              (long)rawMessage.can_id, (unsigned long)rawMessage.can_id);
    logCANError(0xF, "corrupted_can_id");

    // This indicates controller corruption - trigger immediate recovery
    Log.info("CAN controller corruption detected - triggering recovery");
    can_recovery_needed = true;
    return;
  }

  // Check for invalid data length
  if (rawMessage.can_dlc > 8) {
    Log.error("CRITICAL: Invalid CAN DLC: %d", rawMessage.can_dlc);
    logCANError(0xE, "invalid_dlc");
    can_recovery_needed = true;
    return;
  }

  // 1. Parse the raw CAN message
  ParsedCANMessage parsedMsg = canProcessor.parseMessage(rawMessage);

  // 2. Log the message for debugging
  Log.info("CAN message from port %d: type=%s, valid=%s", parsedMsg.sourcePort,
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
    Log.warn("Invalid CAN message received from port %d", parsedMsg.sourcePort);
    logCANError(-1, "message_parsing");
  }
}
// Helper function - just checks, doesn't set flags
bool checkCloudMessages() {
  Particle.process();

  if (pendingCloudCommand) {
    Log.info("Cloud command pending - pausing CAN processing");
    return true; // Exit CAN processing
  }
  return false; // Continue CAN processing
}

void handleCANInterrupt() {
  CAN_INTERRUPT_FLAG = false;

  unsigned long startTime = millis();
  const unsigned long MAX_TIME = 100; // 100ms max processing time
  int messagesProcessed = 0;
  unsigned long lastCloudCheck = millis();

  // Keep reading until empty or timeout
  while ((millis() - startTime) < MAX_TIME) {

    // Check cloud every 20ms OR every 10 messages
    if ((millis() - lastCloudCheck > 20) ||
        (messagesProcessed > 0 && messagesProcessed % 10 == 0)) {

      if (checkCloudMessages()) {
        return; // Exit to handle cloud command
      }
      lastCloudCheck = millis();
    }

    struct can_frame recMsg;

    // Your original read logic - just try to read a message
    int readin = readCanMessage(&recMsg);

    if (readin == ERROR_OK) {
      // YOUR EXISTING VALIDATION CODE - unchanged
      // 1. Basic validity checks
      if ((int32_t)recMsg.can_id < 0 || recMsg.can_id == 0 ||
          recMsg.can_id > MAX_PORTS || recMsg.can_dlc > 8) {
        continue; // Skip this message
      }

      // 2. Data content validation - filter out gibberish
      bool hasValidData = false;
      bool hasGibberish = false;

      for (int i = 0; i < recMsg.can_dlc; i++) {
        uint8_t byte = recMsg.data[i];

        if ((byte >= 0x20 && byte <= 0x7E) || byte == 0x00) {
          hasValidData = true;
        } else if (byte < 0x20 || byte > 0x7E) {
          if (byte != 0x0A && byte != 0x0D && byte != 0x09) {
            hasGibberish = true;
          }
        }
      }

      if (hasGibberish && !hasValidData) {
        continue; // Skip gibberish
      }

      // 3. Message pattern validation
      if (recMsg.can_dlc > 0) {
        char firstChar = (char)recMsg.data[0];
        if (firstChar != 'D' && firstChar != 'K' && firstChar != 'T' &&
            firstChar != 'H' && firstChar != 'U' && firstChar != 'C' &&
            firstChar != 'V' && firstChar != 'F' && firstChar != 'E') {
          continue; // Skip invalid command
        }
      }

      // YOUR EXISTING QUEUE CODE - unchanged
      if (messageCount < CAN_QUEUE_SIZE) {
        incrementMessageCounter();
        messageQueue[queueTail] = recMsg;
        queueTail = (queueTail + 1) % CAN_QUEUE_SIZE;
        messageCount++;
        messagesProcessed++;

        if (messageCount > maxQueueDepth) {
          maxQueueDepth = messageCount;
        }
      } else {
        // Queue full
        queueOverflow = true;
        static uint32_t droppedMessages = 0;
        static unsigned long lastDropLog = 0;
        droppedMessages++;
        totalMessagesDropped++;

        if (millis() - lastDropLog > 5000) {
          Log.info("Queue full - dropped %lu new messages", droppedMessages);
          logCANError(-2, "queue_overflow");
          lastDropLog = millis();
        }
      }

    } else if (readin == mcp2515.ERROR_NOMSG) {
      // No more messages - we've drained the buffer
      break;

    } else {
      // Read error
      static unsigned long lastErrorLog = 0;
      if (millis() - lastErrorLog > 1000) {
        logCANError(readin, "can_read_interrupt");
        lastErrorLog = millis();
      }
      break; // Exit on error
    }
  }

  // Clear any pending interrupts
  mcp2515.clearInterrupts();

  // Check if we need to continue processing
  uint8_t irq = mcp2515.getInterrupts();
  if (irq != 0) {
    CAN_INTERRUPT_FLAG = true; // More messages to process next loop
    if (messagesProcessed > 20) {
      Log.info("CAN burst: processed %d messages, more pending",
               messagesProcessed);
    }
  }

  // Log if we processed a lot of messages (like at startup)
  if (messagesProcessed > 20) {
    Log.info("Processed CAN burst: %d messages", messagesProcessed);
  }

  // Clear interrupts after we're done processing
  mcp2515.clearInterrupts();

  // Check if interrupt line is still asserted
  // (This is just for logging, not to set the flag)
  if (digitalRead(CAN_INT) == LOW) {
    Log.info("CAN interrupt still asserted after processing %d messages",
             messagesProcessed);
    // The hardware will trigger can_interrupt() again naturally
  }

  if (messagesProcessed > 20) {
    Log.info("Processed CAN burst: %d messages", messagesProcessed);
  }
}

void can_interrupt() { CAN_INTERRUPT_FLAG = true; }

// ========================================
// CAN Error Monitoring and Recovery
// ========================================

void performCANRecovery() {
  Log.info("=== PERFORMING EMERGENCY CAN RECOVERY ===");

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
      Log.info("Resetting recovery attempts - %lu ms since last success",
               timeSinceLastSuccess);
      canErrorMonitor.recoveryAttempts = 1; // Reset but count this attempt
      canErrorMonitor.adaptiveMode = true;  // Enable adaptive recovery
    } else {
      Log.info("CRITICAL: Max recovery attempts (%d) exceeded - forcing "
               "device reset",
               MAX_RECOVERY_ATTEMPTS);
      delay(100);
      resetDevice("Max CAN recovery attempts exceeded");
      return;
    }
  }

  Log.info("CAN Recovery attempt %d of %d", canErrorMonitor.recoveryAttempts,
           MAX_RECOVERY_ATTEMPTS);

  // Adaptive recovery delay based on attempt number
  unsigned long recoveryDelay = 500;
  if (canErrorMonitor.adaptiveMode) {
    recoveryDelay =
        1000 + (canErrorMonitor.recoveryAttempts * 500); // Progressive delay
    canErrorMonitor.extendedRecoveryDelay =
        recoveryDelay * 2; // Extended pause after recovery
    Log.info("Adaptive recovery mode - using %lu ms delay", recoveryDelay);
  }

  delay(recoveryDelay); // Give system time to stabilize

  // Reset MCP2515 controller
  Log.info("Resetting MCP2515 controller...");
  int err = mcp2515.reset();
  if (err != mcp2515.ERROR_OK) {
    Log.info("CRITICAL: MCP2515 reset failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 reset failed");
    return;
  }

  delay(200);

  // Reconfigure controller
  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != mcp2515.ERROR_OK) {
    Log.info("CRITICAL: MCP2515 bitrate config failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 config failed");
    return;
  }

  delay(100);

  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK) {
    Log.info("CRITICAL: MCP2515 normal mode failed - forcing device reset");
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
    Log.info("CAN recovery complete - interrupt re-enabled with clean state");
  } else {
    canInterruptAttached = false; // Disable polling until connected
    Log.info(
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

  Log.info("=== CAN RECOVERY COMPLETED SUCCESSFULLY ===");
  Log.info("Recovery attempt %d succeeded", canErrorMonitor.recoveryAttempts);

  // Extended pause in adaptive mode for system stabilization
  unsigned long postRecoveryDelay = 1000;
  if (canErrorMonitor.adaptiveMode &&
      canErrorMonitor.extendedRecoveryDelay > 1000) {
    postRecoveryDelay = canErrorMonitor.extendedRecoveryDelay;
    Log.info("Extended post-recovery delay: %lu ms", postRecoveryDelay);
  }

  delay(postRecoveryDelay);
}

void logCANError(int errorCode, const char *operation) {
  unsigned long currentTime = millis();

  canErrorMonitor.totalErrors++;
  canErrorMonitor.lastErrorTime = currentTime;
  canErrorMonitor.consecutiveErrors++;

  Log.info("CAN Error #%lu: %s failed with code %d (consecutive: %d)",
           canErrorMonitor.totalErrors, operation, errorCode,
           canErrorMonitor.consecutiveErrors);

  // Check for specific error codes that indicate controller corruption
  if (errorCode == 0xA || errorCode == 10 || errorCode == 0xF ||
      errorCode == 15) {
    Log.info("CRITICAL: Detected ERROR_A/ERROR_F - MCP2515 controller "
             "corruption!");
    Log.info("Triggering immediate CAN recovery to prevent system freeze");
    can_recovery_needed = true;

    // If we've seen these errors multiple times, force device reset
    if (canErrorMonitor.consecutiveErrors >= 2) {
      Log.info("Multiple controller corruption errors - forcing device reset");
      delay(100);
      resetDevice("MCP2515 controller corruption");
      return;
    }
  }

  // Lower threshold for faster recovery during error cascades
  if (canErrorMonitor.consecutiveErrors >= 3) {
    Log.info("Fast recovery trigger - %d consecutive errors",
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
    Log.info("CAN operation successful - resetting error count");
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
      Log.info("Resetting recovery attempt counter after sustained "
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
    Log.info("CAN BUS ERROR %s", operation);
    ReturnErrorString(err, ret, sizeof(ret));
    sprintf(can_err_msg, "CAN BUS ERROR %s: %s", operation, ret);
    Log.info("%s", can_err_msg);

    // Log this as a CAN error for monitoring
    logCANError(err, operation);
  }
}

int resetDevice(String command) {
  if (command.length() > 0) {
    Log.info("Device reset requested. Reason: %s", command.c_str());
    Log.info("Free memory before reset: %lu bytes", System.freeMemory());
    Log.info("System uptime: %lu ms", millis());
    Log.info("Cellular connected: %s", CELLULAR_CONNECTED ? "yes" : "no");
    Log.info("Credentials valid: %s", areCredentialsValid() ? "yes" : "no");
    Log.info("CAN errors in last period: %d", can_error_count);
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

  Log.info("[%lu ms] Checkpoint: %s (Delta: %lu ms)", now, checkpoint,
           now - lastLogTime);

  lastLogTime = now;
}

void logResetReason() {
  int reason = System.resetReason();
  int reasonData = System.resetReasonData();

  Log.info("RESET REASON: %d", reason);
  Log.info("RESET REASON DATA: %d", reasonData);

  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Reason: %d, Data: %d", reason, reasonData);

  Particle.publish("RESET REASON", buffer, PRIVATE);
}

void checkSystemHealth() {
  unsigned long freeMemory = System.freeMemory();
  unsigned long uptime = millis();

  static unsigned long lastHealthCheck = 0;
  if (uptime - lastHealthCheck > 60000) {
    // Calculate message drop rate
    float dropRate = 0.0;
    if (totalMessagesProcessed + totalMessagesDropped > 0) {
      dropRate = (totalMessagesDropped * 100.0) /
                 (totalMessagesProcessed + totalMessagesDropped);
    }

    Log.info("System Health - Uptime: %lu min, Free Memory: %lu bytes",
             uptime / 60000, freeMemory);
    Log.info(
        "Interrupt Queue: %d/%d | Processing Queue: %d/%d | Max Depth: %lu",
        messageCount, CAN_QUEUE_SIZE, processingQueueCount,
        PROCESSING_QUEUE_SIZE, maxQueueDepth);
    Log.info("Messages - Processed: %lu | Dropped: %lu (%.1f%%)",
             totalMessagesProcessed, totalMessagesDropped, dropRate);
    Log.info("Cellular Status: %s",
             CELLULAR_CONNECTED ? "connected" : "disconnected");
    Log.info("Credentials: %s", getCredentialsStatus().c_str());
    Log.info("CAN Errors (last minute): %d", can_error_count);
    Log.info("CAN Recovery needed: %s", can_recovery_needed ? "yes" : "no");

    // Warn if drop rate is concerning
    if (dropRate > 5.0) {
      Log.info("WARNING: High message drop rate detected: %.1f%%", dropRate);
    }

    if (portFlagHandler) {
      Log.info("Ports with pending flags: %d",
               portFlagHandler->getPendingPortsCount());
    }

    lastHealthCheck = uptime;
  }
}

void emergencyReset(const char *reason) {
  Log.info("EMERGENCY RESET: %s", reason);
  Log.info("Error Stats - Consecutive: %d, Total: %lu",
           canErrorMonitor.consecutiveErrors, canErrorMonitor.totalErrors);
  delay(200); // Ensure message is sent
  System.reset();
}

// Internet connectivity monitoring thread with safety features
// This thread monitors internet connection and performs recovery actions:
// 1. Soft recovery (disconnect/reconnect) after 30 seconds
// 2. Hard reset after 1 minute, but only if system is in safe state
void internetCheckThread() {
  static bool did_disconnect = false;
  static unsigned long disconnectTime = 0;
  static unsigned long lastCheckTime = 0;
  while (true) {
    // Non-blocking check every 5 seconds
    if (millis() - lastCheckTime > 5000) {

      lastCheckTime = millis();

      CELLULAR_CONNECTED = (Cellular.ready() && Particle.connected());

      if (!CELLULAR_CONNECTED) {
        if (!did_disconnect) { // Only set disconnectTime on FIRST disconnect
          Log.info("Internet disconnected - starting recovery timeline");
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

          Log.info("RESET: Internet disconnected for %lu ms - "
                   "performing device reset",
                   disconnectedDuration);
          System.reset(RESET_NO_WAIT);
        }
      } else {
        if (did_disconnect) {
          // Connection restored
          Log.info("Internet reconnected after %lu ms offline",
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

  Log.info("CAN Health Monitor thread started");

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
          Log.info("Resetting CAN error count (was %d)", can_error_count);
        }
        can_error_count = 0;
        last_can_error_time = currentTime;
      }

      // Check if error rate is too high
      if (can_error_count > MAX_CAN_ERRORS_PER_MINUTE) {
        Log.info("CAN error rate too high (%d errors/minute), "
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
          Log.info("CAN Error Threshold Reached: %d consecutive errors",
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
          Log.info("CAN Error Window Expired - Resetting "
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
          Log.info("No recent CAN success but hardware looks healthy - "
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
            Log.info("Gentle restart failed, CAN may have deeper issues");
            emergencyReset("CAN operations failed even after gentle restart");
          }
        } else {
          // Actual CAN hardware errors detected
          Log.info("CAN hardware errors detected (flags: 0x%02X)",
                   currentFlags);
          emergencyReset("No successful CAN operations with hardware errors");
        }
      }

      // Hardware watchdog handles main loop freeze detection automatically

      // Check CAN error rate with lower threshold
      if (canErrorMonitor.consecutiveErrors >= 6) {
        Log.info("CRITICAL: Excessive CAN errors (%d), forcing reset",
                 canErrorMonitor.consecutiveErrors);
        delay(100);
        emergencyReset("Excessive CAN errors");
      }

      // Check for queue overflow conditions
      if (queueOverflow) {
        Log.info("CAN message queue overflow detected");
        queueOverflow = false; // Reset flag
        logCANError(-3, "persistent_queue_overflow");
      }

      // Periodic queue health check - just monitor, don't clear
      if (messageCount > (CAN_QUEUE_SIZE * 2 / 3)) { // More than 66% full
        static unsigned long lastWarningTime = 0;
        static int stuckCount = 0;
        static int lastQueueCount = 0;

        if (currentTime - lastWarningTime > 10000) { // Log every 10 seconds
          Log.info("WARNING: CAN queue filling up (%d/%d messages)",
                   messageCount, CAN_QUEUE_SIZE);
          lastWarningTime = currentTime;
        }

        // Detect if queue is stuck but DON'T clear messages
        if (messageCount == lastQueueCount &&
            messageCount > (CAN_QUEUE_SIZE * 4 / 5)) {
          stuckCount++;
          if (stuckCount > 3) {
            Log.info("CRITICAL: Queue appears stuck at %d messages - may need "
                     "recovery",
                     messageCount);
            // Instead of clearing, try to trigger recovery
            canErrorMonitor.consecutiveErrors++;
            stuckCount = 0;
          }
        } else {
          stuckCount = 0; // Reset if queue is moving
        }
        lastQueueCount = messageCount;
      }

      // Report RX overflow statistics if any have occurred
      if (canErrorMonitor.rxOverflowCount > 0) {
        unsigned long overflowDuration =
            currentTime - canErrorMonitor.firstRxOverflowTime;
        Log.info("RX Overflow Status: %d overflows in %lu ms (last "
                 "clear: %lu ms ago)",
                 canErrorMonitor.rxOverflowCount, overflowDuration,
                 currentTime - canErrorMonitor.lastRxOverflowClear);

        // Reset overflow tracking after successful recovery (if no overflow for
        // 10 seconds and system is healthy)
        if (currentTime - canErrorMonitor.lastRxOverflowClear > 10000 &&
            canErrorMonitor.consecutiveErrors == 0) {
          Log.info(
              "RX overflow resolved - resetting tracking (was %d overflows)",
              canErrorMonitor.rxOverflowCount);
          canErrorMonitor.rxOverflowCount = 0;
          canErrorMonitor.firstRxOverflowTime = 0;
        }
      }

      // Check for invalid CAN messages (indicates controller corruption)
      if (canErrorMonitor.consecutiveErrors >= 3) {
        Log.info("CAN controller corruption suspected (%d consecutive errors)",
                 canErrorMonitor.consecutiveErrors);
      }

      // Log periodic health status during error conditions
      // Only log health status every 10 seconds to reduce noise
      static unsigned long lastHealthLog = 0;
      if ((canErrorMonitor.consecutiveErrors > 0 ||
           canErrorMonitor.inRecoveryMode) &&
          currentTime - lastHealthLog > 10000) {
        Log.info("CAN Health: Errors=%d, Attempts=%d, Recovering=%s, "
                 "HW_Flags=0x%02X",
                 canErrorMonitor.consecutiveErrors,
                 canErrorMonitor.recoveryAttempts,
                 canErrorMonitor.inRecoveryMode ? "YES" : "NO", currentFlags);
        lastHealthLog = currentTime;
      }

      // // Log CAN error flags if any are detected (less frequent)
      // static unsigned long lastFlagLog = 0;
      // if (currentFlags != 0 && currentTime - lastFlagLog > 30000) //
      // Every 30 seconds
      // {
      //   Log.info("MCP2515 hardware error flags detected: 0x%02X",
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
      Log.info("Interrupt health monitoring disabled - system not fully "
               "operational");
      wasReady = false;
    }
    lastInterruptTime = currentTime; // Reset timer to prevent false alarms
    return;
  }

  // Log when monitoring becomes active
  static bool wasReady = false;
  if (!wasReady) {
    Log.info("Interrupt health monitoring enabled - system fully operational");
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
    Log.info("No CAN message processing for %lu ms (timeout: %lu ms) - "
             "checking interrupt system",
             timeSinceLastInterrupt, dynamicTimeout);

    // Check MCP2515 interrupt flags to see if they're stuck
    uint8_t intFlags = mcp2515.getInterrupts();
    Log.info("MCP2515 interrupt flags: 0x%02X", intFlags);

    if (intFlags != 0) {
      Log.info("CRITICAL: MCP2515 has pending interrupts but no message "
               "processing!");

      // Try to recover the interrupt system
      recoverInterruptSystem();
    } else {
      Log.info("MCP2515 no pending interrupts - may be normal quiet period");
      // Update last interrupt time to prevent false alarms during quiet
      // periods
      lastInterruptTime = currentTime - (INTERRUPT_TIMEOUT / 2);
    }
  }
}

void recoverInterruptSystem() {
  Log.info("=== ATTEMPTING INTERRUPT SYSTEM RECOVERY ===");

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
    Log.info("Interrupt re-attached after recovery with cleared state");
  } else {
    canInterruptAttached = false; // Disable polling until connected
    Log.info("Interrupt not re-attached - waiting for connection");
  }

  // Reset interrupt health tracking
  lastInterruptTime = millis();
  interruptHealthy = true;

  Log.info("Interrupt system recovery completed");
  Log.info("Re-attached interrupt on pin %d", CAN_INT);

  // Test interrupt by reading any pending messages
  uint8_t intFlags = mcp2515.getInterrupts();
  if (intFlags != 0) {
    Log.info("Post-recovery: MCP2515 flags: 0x%02X", intFlags);
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
      Log.info("TX/RX monitoring disabled - system not ready");
      txRxWasReady = false;
    }
    lastTransmissionTime = currentTime;
    lastInterruptTime = currentTime;
    return;
  }

  // Log when TX/RX monitoring becomes active
  static bool txRxWasReady = false;
  if (!txRxWasReady) {
    Log.info("TX/RX monitoring enabled - system ready");
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
      Log.info("CRITICAL: TX/RX imbalance - TX:%lu ms ago, RX:%lu ms "
               "ago (timeout: %lu ms)",
               timeSinceLastTX, timeSinceLastRX, dynamicTxRxTimeout);
      Log.info("TX works but RX failed - attempting interrupt recovery");
      recoverInterruptSystem();
      lastInterruptRecovery = currentTime;
    } else {
      // Reset timer to prevent constant triggering
      lastInterruptTime = currentTime - (dynamicTxRxTimeout / 2);
    }
  }
}

void prepareCANForInterrupt() {
  // Check if this is the first startup (not a recovery)
  static bool isFirstStartup = true;

  if (isFirstStartup) {
    Log.info("INITIAL STARTUP: Aggressively clearing ALL CAN buffers...");

    // 1. Make sure interrupt is detached
    detachInterrupt(CAN_INT);
    delay(100); // Longer delay for initial startup

    // 2. Clear ALL error flags multiple times
    for (int i = 0; i < 3; i++) {
      mcp2515.clearRXnOVR();
      mcp2515.clearInterrupts();
      uint8_t errorFlags = mcp2515.getErrorFlags();
      if (errorFlags != 0) {
        Log.info("Clearing error flags (pass %d): 0x%02X", i + 1, errorFlags);
      }
      delay(10);
    }

    // 3. Aggressively read and discard ALL messages
    can_frame dummyMsg;
    int totalCleared = 0;
    int passes = 0;
    const int MAX_MESSAGES =
        500; // Handle worst case: all ports sending multiple messages

    // Keep clearing until truly empty
    while (passes < 10) { // Up to 10 passes to ensure completely empty
      int messagesThisPass = 0;

      // Clear as many messages as possible
      while (mcp2515.readMessage(&dummyMsg) == MCP2515::ERROR_OK &&
             totalCleared < MAX_MESSAGES) {
        totalCleared++;
        messagesThisPass++;
      }

      if (messagesThisPass == 0) {
        // No messages found, we're done
        break;
      }

      Log.info("Startup clear pass %d: removed %d messages", passes + 1,
               messagesThisPass);

      // Clear any overflow flags that occurred during clearing
      mcp2515.clearRXnOVR();
      mcp2515.clearInterrupts();

      passes++;
      delay(20); // Brief delay between passes
    }

    Log.info("INITIAL STARTUP COMPLETE: Cleared %d total stale messages in %d "
             "passes",
             totalCleared, passes);

    isFirstStartup = false; // Mark that we've done the aggressive clear

    // 4. Final cleanup
    mcp2515.clearRXnOVR();
    mcp2515.clearInterrupts();

  } else {
    // Normal (non-initial) preparation for recovery scenarios
    Log.info("Preparing CAN for interrupt attachment...");

    detachInterrupt(CAN_INT);
    delay(50);

    // Clear error flags
    uint8_t errorFlags = mcp2515.getErrorFlags();
    if (errorFlags != 0) {
      Log.info("Clearing MCP2515 error flags: 0x%02X", errorFlags);
      mcp2515.clearRXnOVR();
    }

    // Read and discard pending messages (less aggressive for recovery)
    can_frame dummyMsg;
    int messagesCleared = 0;
    while (mcp2515.readMessage(&dummyMsg) == MCP2515::ERROR_OK &&
           messagesCleared < 50) {
      messagesCleared++;
    }
    if (messagesCleared > 0) {
      Log.info("Discarded %d pending messages from MCP2515", messagesCleared);
    }

    // Clear interrupt flags
    uint8_t intFlags = mcp2515.getInterrupts();
    if (intFlags != 0) {
      Log.info("Clearing MCP2515 interrupt flags: 0x%02X", intFlags);
      mcp2515.clearInterrupts();
    }
  }

  // 5. Reset our software queue state
  noInterrupts();
  queueHead = 0;
  queueTail = 0;
  messageCount = 0;
  queueOverflow = false;
  memset(messageQueue, 0, sizeof(messageQueue));
  interrupts();

  Log.info("Software queues reset");

  // 6. Reset CAN error monitoring state
  canErrorMonitor.consecutiveErrors = 0;
  canErrorMonitor.rxOverflowCount = 0;
  canErrorMonitor.firstRxOverflowTime = 0;
  canErrorMonitor.lastRxOverflowClear = millis();

  // 7. Re-attach interrupt
  pinMode(CAN_INT, INPUT_PULLUP);
  attachInterrupt(CAN_INT, can_interrupt, FALLING);
  canInterruptAttached = true; // Enable port polling

  Log.info("CAN interrupt re-attached with clean state");
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

    // 5ms chunks for faster cloud command detection
    delay(5);
  }
}

void outOfMemoryHandler(system_event_t event, int param) {
  // Log to retained memory for next boot
  oomTracker.wasOutOfMemory = true;
  oomTracker.outOfMemorySize = param;
  oomTracker.freeMemoryAtRestart = System.freeMemory();
  oomTracker.uptimeAtOOM = millis();

  // // IMMEDIATE reset - don't wait for main loop
  // Log.error("CRITICAL: Out of memory (size=%d, free=%lu) - immediate reset",
  //           param, System.freeMemory());
  delay(10); // Brief delay for message transmission and retained memory write
  System.reset(RESET_NO_WAIT);
}

void handleRxOverflowWithEscalation(unsigned long currentTime) {
  // Track RX overflow occurrences
  canErrorMonitor.rxOverflowCount++;

  // Initialize tracking on first overflow
  if (canErrorMonitor.rxOverflowCount == 1) {
    canErrorMonitor.firstRxOverflowTime = currentTime;
  }

  Log.info("RX buffer overflow detected (#%d) - clearing buffers",
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
      Log.info("CRITICAL: RX overflow persisting despite buffer clearing");
      Log.info("Overflow count: %d in %lu ms", canErrorMonitor.rxOverflowCount,
               overflowDuration);
      Log.info("Buffer clearing ineffective - triggering system restart");

      // Log the critical condition
      logCANError(-11, "persistent_rx_overflow_restart");

      // Force immediate restart
      Log.info("*** FORCING SYSTEM RESTART DUE TO PERSISTENT RX OVERFLOW ***");
      delay(100); // Brief delay for serial output
      System.reset(RESET_NO_WAIT);
    }
  }
}
