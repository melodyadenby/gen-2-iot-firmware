#include "Arduino.h"
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

// Watchdog Configuration
const unsigned long WATCHDOG_TIMEOUT = 30000; // 30 seconds
unsigned long last_watchdog_feed = 0;
bool watchdog_enabled = false;

// CAN Error Monitoring Constants
const int CAN_ERROR_THRESHOLD = 5;            // Max consecutive errors before recovery
const unsigned long CAN_ERROR_WINDOW = 10000; // 10 seconds error window
const unsigned long CAN_RECOVERY_DELAY =
    5000;                             // 5 seconds between recovery attempts
const int MAX_RECOVERY_ATTEMPTS = 10; // Max recovery attempts before reset
const unsigned long RECOVERY_SUCCESS_RESET_TIME =
    300000; // 5 minutes of success resets recovery count

// CAN Error Monitor Structure
struct CANErrorMonitor
{
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
} canErrorMonitor = {0, 0, 0, 0, false, 0, 0, 0, false, 0};

can_frame messageQueue[50];

// Global hardware watchdog object
ApplicationWatchdog *hardwareWatchdog;

// Hardware watchdog handler
void hardwareWatchdogHandler()
{
  // Do minimal work here - just reset the system
  // Don't use Serial.print, Particle.publish, etc.
  System.reset(RESET_NO_WAIT);
}

void setup()
{
  initializeSystem();

  // Initialize hardware watchdog early - 60 second timeout
  hardwareWatchdog =
      new ApplicationWatchdog(60000, hardwareWatchdogHandler, 1536);
  Serial.printlnf("Hardware ApplicationWatchdog initialized (60s timeout)");

  initializeArchitecture();
  initializeHardware();

  if (!CAN_ERROR)
  {
    initializeParticle();
    requestCredentials();

    // Initialize recovery state
    canErrorMonitor.lastSuccessTime = millis();
    canErrorMonitor.recoveryAttempts = 0;

    // Enable watchdog after successful initialization
    enableWatchdog();

    Serial.printlnf("=== SYSTEM STARTUP COMPLETE ===");
    Serial.printlnf("CAN Error Monitoring: ENABLED");
    Serial.printlnf("Watchdog Timer: ENABLED");
    Serial.printlnf("Recovery System: READY");
  }
  else
  {
    Serial.printlnf("=== STARTUP FAILED - CAN ERROR DETECTED ===");
    resetDevice("CAN error during startup");
  }
}

void loop()
{
  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();

  // Detect if main loop is running too slowly (potential freeze indicator)
  if (lastLoopTime > 0 && (currentTime - lastLoopTime) > 5000)
  {
    Serial.printlnf("WARNING: Main loop delay detected: %lu ms",
                    currentTime - lastLoopTime);
  }
  lastLoopTime = currentTime;

  handleSystemLoop();
  checkWatchdog();
  checkCANHealth();
  feedWatchdog();                 // Feed watchdog at end of successful loop iteration
  ApplicationWatchdog::checkin(); // Feed hardware watchdog

  // Small delay to prevent CPU overload during error conditions
  if (CAN_ERROR || can_recovery_needed)
  {
    delay(10);
  }
}

void initializeArchitecture()
{
  // Initialize the clean architecture components
  portEventHandler =
      new PortEventHandler(nullptr); // Will use global port state functions
  portFlagHandler =
      new PortFlagHandler(nullptr); // Will use global port state functions

  Serial.printlnf("Architecture components initialized");
}

void initializeSystem()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(2000);

  // Initialize all subsystems
  initializePorts();
  initializeCredentials();
  initializeMQTT();

  Particle.variable("CAN_ERROR", CAN_ERROR);
  Particle.variable("pub_id", MANUAL_MODE, STRING);
  Particle.variable("MQTT_connected", BROKER_CONNECTED);
  Particle.variable("credentialsFetched", credentialsFetched);
  Particle.variable("total_messages_received", total_messages_received);

  Serial.printlnf("*** KUHMUTE IoT V %s ***", BUILD_VERSION);
  Serial.printlnf("Device ID: %s", Particle.deviceID().c_str());
  Serial.printlnf("Environment: %s", getCurrentEnvironment());
  Serial.printlnf("System initialized");
}

void initializeHardware()
{
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
  if (err != mcp2515.ERROR_OK)
  {
    reportCANError(err, "reset", true);
    return;
  }

  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != mcp2515.ERROR_OK)
  {
    reportCANError(err, "setBitrate", true);
    return;
  }

  delay(50);
  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK)
  {
    reportCANError(err, "setNormalMode", true);
    return;
  }

  pinMode(CAN_INT, INPUT_PULLUP);
  attachInterrupt(CAN_INT, can_interrupt, FALLING);

  // Start CAN processing thread
  new Thread("can_thread", canThread);
  new Thread("port_request_thread", port_request_thread);
  new Thread("can_monitor_thread", canMonitorThread);
  new Thread("can_health_monitor", canHealthMonitorThread);

  Serial.printlnf("Hardware initialized");
}

void initializeParticle()
{
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
  if (!Particle.connected())
  {
    Serial.printlnf("Failed to connect to Particle Cloud");
    resetDevice("Cloud connection timeout");
    return;
  }

  // Log reset reason
  logResetReason();

  setLightBlue();
  Serial.printlnf("Particle Cloud connected");
}

void handleSystemLoop()
{
  // Feed watchdog FIRST
  feedWatchdog();

  // Handle CAN recovery IMMEDIATELY - before anything else
  if (can_recovery_needed)
  {
    Serial.printlnf("EMERGENCY: Executing CAN recovery due to error cascade");
    performCANRecovery();
    delay(1000); // Give recovery time to complete
    return;      // Exit immediately, don't continue with normal operations
  }

  // Handle critical errors
  if (CAN_ERROR)
  {
    blinkCANError();
    feedWatchdog(); // Keep feeding watchdog during error state
    delay(100);     // Prevent tight loop
    return;
  }

  // Check for credential fetch failures
  if (attemptedCredentialsFetchCount > MAX_CREDENTIAL_ATTEMPTS)
  {
    Serial.printlnf("Failed to fetch credentials after max attempts");
    blinkIdentityError();
    return;
  }

  // Handle connection states
  if (!Particle.connected())
  {
    setLightRed();
    return;
  }
  // Handle port data requests (staggered polling)
  handlePortDataRequests();

  // Handle MQTT and credentials
  handleMQTTClientLoop();
  handleCredentials();
  updateSystemStatus();

  // System health monitoring
  checkSystemHealth();

  // Small delay to prevent CPU overload
  delay(5);
}

/**
 * Handle systematic port data requests with staggered polling
 * Polls each port in sequence with delays to avoid overwhelming the CAN bus
 */
void handlePortDataRequests()
{
  // EMERGENCY STOP - Don't poll if we're in error cascade
  if (can_recovery_needed || CAN_ERROR)
  {
    delay(100); // Prevent tight loop during recovery
    return;
  }

  // Add consecutive failure tracking per port
  static int portFailureCount[MAX_PORTS + 1] = {0}; // Track failures per port
  static bool pollingDisabled = false;
  static unsigned long pollingDisabledTime = 0;

  // Check if we should re-enable polling after errors cleared
  if (pollingDisabled)
  {
    unsigned long waitTime = 5000; // Default 5 seconds

    // Extended wait time in adaptive mode
    if (canErrorMonitor.adaptiveMode &&
        canErrorMonitor.extendedRecoveryDelay > 5000)
    {
      waitTime = canErrorMonitor.extendedRecoveryDelay;
    }

    if (canErrorMonitor.consecutiveErrors == 0 &&
        millis() - pollingDisabledTime > waitTime)
    {
      Serial.printlnf(
          "Re-enabling port polling - errors have cleared (waited %lu ms)",
          waitTime);
      pollingDisabled = false;
      // Reset all port failure counts
      for (int i = 0; i <= MAX_PORTS; i++)
      {
        portFailureCount[i] = 0;
      }
    }
    else
    {
      delay(100);
      return;
    }
  }

  unsigned long current_time = millis();
  if (current_time - last_port_check_reset >= PORT_CHECK_INTERVAL)
  {
    markPortsUnpolled();
    last_port_check_reset = current_time; // Update the last reset time
    Serial.println("DID_PORT_CHECK reset for all ports.");
  }

  static int current_poll_port = 1;
  static unsigned long last_poll_send_time = 0;
  const unsigned long POLL_STAGGER_DELAY = 300; // 1s between requests

  // Check if enough time has passed since last poll
  if (current_time - last_poll_send_time < POLL_STAGGER_DELAY)
  {
    return;
  }

  // Check if portFlagHandler is available
  if (!portFlagHandler)
  {
    Serial.printlnf("portFlagHandler not initialized");
    return;
  }

  // Find next port that needs polling
  for (int attempts = 0; attempts < MAX_PORTS; attempts++)
  {
    if (current_poll_port > MAX_PORTS)
    {
      current_poll_port = 1; // Wrap around
    }

    if (!hasPortBeenPolled(current_poll_port))
    {
      bool success = portFlagHandler->sendGetPortData(current_poll_port);

      if (success)
      {
        markPortPolled(current_poll_port);
        // Reset failure count on success
        portFailureCount[current_poll_port] = 0;
        resetCANSuccessCounter();
      }
      else
      {
        portFailureCount[current_poll_port]++;
        Serial.printlnf("Failed to poll port %d (failures: %d)",
                        current_poll_port, portFailureCount[current_poll_port]);

        // Skip this port if it's failing repeatedly
        if (portFailureCount[current_poll_port] >= 3)
        {
          Serial.printlnf(
              "Port %d has failed %d times - skipping for this cycle",
              current_poll_port, portFailureCount[current_poll_port]);
          markPortPolled(current_poll_port);       // Mark as "polled" to skip it
          portFailureCount[current_poll_port] = 0; // Reset for next cycle
        }

        logCANError(-1, "port_polling");

        // If too many consecutive errors, trigger IMMEDIATE recovery
        if (canErrorMonitor.consecutiveErrors >= 3)
        {
          Serial.printlnf("CRITICAL: Immediate CAN recovery needed");
          can_recovery_needed = true;
          pollingDisabled = true;
          pollingDisabledTime = millis();
          return; // Stop polling immediately
        }

        // Feed watchdog during error conditions to prevent timeout
        feedWatchdog();
      }

      last_poll_send_time = current_time;
      current_poll_port++; // Move to next port for next iteration
      break;               // Only send one request per cycle
    }
    current_poll_port++;
  }
}

void handleCredentials()
{
  if (areCredentialsValid())
  {
    return; // Already have valid credentials
  }

  if (shouldRetryCredentials())
  {
    requestCredentials();
  }
}

void updateSystemStatus()
{
  if (areCredentialsValid() && isMQTTConnected())
  {
    setLightGreen(); // All systems operational
  }
  else if (areCredentialsValid())
  {
    setLightPurple(); // Have credentials, connecting to MQTT
  }
  else
  {
    setLightBlue(); // Fetching credentials
  }
}

// ========================================
// CAN Processing
// ========================================

void canThread()
{
  Serial.printlnf("CAN processing thread started");

  while (true)
  {
    if (areCredentialsValid() && isMQTTConnected() && Particle.connected() &&
        !CAN_ERROR && !can_recovery_needed)
    {
      // Process incoming CAN messages
      handleCanQueue();

      // Process port flags (replaces old flagHandler)
      if (portFlagHandler)
      {
        portFlagHandler->processAllPortFlags();
      }

      // Small delay to prevent busy-waiting
      delay(10);
    }
    else
    {
      // Small delay to prevent busy-waiting
      delay(10);
    }

    // Feed hardware watchdog during port request thread
    ApplicationWatchdog::checkin();
  }
}

void port_request_thread()
{
  while (true)
  {
    if (areCredentialsValid() && isMQTTConnected() && Particle.connected() &&
        !CAN_ERROR && !can_recovery_needed)
    {
      handlePortDataRequests();

      // Small delay to prevent busy-waiting
      delay(10);
    }
    else
    {
      // Small delay to prevent busy-waiting
      delay(10);
    }

    // Feed hardware watchdog during CAN thread
    ApplicationWatchdog::checkin();
  }
}

void handleCanQueue()
{
  int messagesProcessed = 0;
  const int MAX_MESSAGES_PER_LOOP = 8;

  while (messageCount > 0 && messagesProcessed < MAX_MESSAGES_PER_LOOP)
  {
    can_frame msg;
    bool validMessage = false;

    // Thread-safe message extraction
    noInterrupts();
    if (messageCount > 0)
    {
      if (queueHead >= 0 && queueHead < 50 && queueTail >= 0 &&
          queueTail < 50)
      {
        msg = messageQueue[queueHead];
        queueHead = (queueHead + 1) % 50;
        messageCount--;
        validMessage = true;

        // Handle queue overflow reset
        if (queueOverflow && messageCount < 25)
        {
          queueOverflow = false;
          Serial.println("Queue overflow cleared");
        }
      }
      else
      {
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
    if (validMessage)
    {
      processCANMessage(msg);
      messagesProcessed++;
    }

    // Yield periodically
    if (messagesProcessed % 2 == 0)
    {
      Particle.process();
    }
  }
}

void processCANMessage(const can_frame &rawMessage)
{
  // Check for corrupted CAN ID (negative or excessively large values)
  if ((int32_t)rawMessage.can_id < 0 || rawMessage.can_id > 16777215)
  {
    Serial.printlnf("CRITICAL: Corrupted CAN ID detected: %ld (0x%lX)",
                    (long)rawMessage.can_id, (unsigned long)rawMessage.can_id);
    logCANError(0xF, "corrupted_can_id");

    // This indicates controller corruption - trigger immediate recovery
    Serial.printlnf("CAN controller corruption detected - triggering recovery");
    can_recovery_needed = true;
    return;
  }

  // Check for invalid data length
  if (rawMessage.can_dlc > 8)
  {
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
  if (parsedMsg.isValid && portEventHandler)
  {
    portEventHandler->handleCANMessage(parsedMsg);
    // Reset consecutive error count on successful message processing
    resetCANSuccessCounter();
  }
  else if (!parsedMsg.isValid)
  {
    Serial.printlnf("Invalid CAN message received from port %d",
                    parsedMsg.sourcePort);
    logCANError(-1, "message_parsing");
  }
}

void can_interrupt()
{
  // Minimal interrupt handler - just queue the message
  struct can_frame recMsg;

  // Read the message
  int readin = readCanMessage(&recMsg);

  if (readin == ERROR_OK)
  {
    // Comprehensive corruption and gibberish filtering

    // 1. Basic validity checks
    if ((int32_t)recMsg.can_id < 0 || recMsg.can_id == 0 ||
        recMsg.can_id > MAX_PORTS || recMsg.can_dlc > 8)
    {
      // Don't queue obviously corrupted messages
      return; // Silent discard for performance
    }

    // 2. Data content validation - filter out gibberish
    bool hasValidData = false;
    bool hasGibberish = false;

    for (int i = 0; i < recMsg.can_dlc; i++)
    {
      uint8_t byte = recMsg.data[i];

      // Check for valid ASCII characters (printable range)
      if ((byte >= 0x20 && byte <= 0x7E) || byte == 0x00)
      {
        hasValidData = true;
      }
      // Check for obvious gibberish (control chars, high ASCII)
      else if (byte < 0x20 || byte > 0x7E)
      {
        // Allow some common control characters
        if (byte != 0x0A && byte != 0x0D && byte != 0x09) // LF, CR, TAB
        {
          hasGibberish = true;
        }
      }
    }

    // Reject if mostly gibberish or no valid data
    if (hasGibberish && !hasValidData)
    {
      return; // Silent discard of gibberish
    }

    // 3. Message pattern validation - must start with valid command
    if (recMsg.can_dlc > 0)
    {
      char firstChar = (char)recMsg.data[0];
      // Valid message types: D(status), K(VIN), T(temp), H(heartbeat), etc.
      if (firstChar != 'D' && firstChar != 'K' && firstChar != 'T' &&
          firstChar != 'H' && firstChar != 'U' && firstChar != 'C' &&
          firstChar != 'V' && firstChar != 'F' && firstChar != 'E')
      {
        return; // Silent discard of invalid command
      }
    }

    // Add to queue if there's space
    if (messageCount < 50)
    {
      incrementMessageCounter();
      messageQueue[queueTail] = recMsg;
      queueTail = (queueTail + 1) % 50;
      messageCount++;
    }
    else
    {
      queueOverflow = true;
      logCANError(-2, "queue_overflow");
    }
  }
  else
  {
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

void canMonitorThread()
{
  Serial.printlnf("CAN monitor thread started");

  while (true)
  {
    // Monitor CAN error rate
    unsigned long currentTime = millis();

    // Reset error count every minute
    if (currentTime - last_can_error_time > CAN_ERROR_RESET_INTERVAL)
    {
      if (can_error_count > 0)
      {
        Serial.printlnf("Resetting CAN error count (was %d)", can_error_count);
      }
      can_error_count = 0;
      last_can_error_time = currentTime;
    }

    // Check if error rate is too high
    if (can_error_count > MAX_CAN_ERRORS_PER_MINUTE)
    {
      Serial.printlnf(
          "CAN error rate too high (%d errors/minute), triggering recovery",
          can_error_count);
      can_recovery_needed = true;
    }

    // Feed hardware watchdog during monitoring
    ApplicationWatchdog::checkin();

    delay(1000); // Check every second
  }
}

void performCANRecovery()
{
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
  interrupts();

  // Update recovery tracking
  canErrorMonitor.recoveryAttempts++;
  canErrorMonitor.inRecoveryMode = true;
  canErrorMonitor.lastRecoveryAttempt = millis();

  // Check if we've exceeded max recovery attempts without recent success
  if (canErrorMonitor.recoveryAttempts >= MAX_RECOVERY_ATTEMPTS)
  {
    unsigned long timeSinceLastSuccess =
        millis() - canErrorMonitor.lastSuccessfulRecovery;

    // If it's been a long time since last successful recovery, allow reset of
    // attempts
    if (timeSinceLastSuccess > RECOVERY_SUCCESS_RESET_TIME)
    {
      Serial.printlnf("Resetting recovery attempts - %lu ms since last success",
                      timeSinceLastSuccess);
      canErrorMonitor.recoveryAttempts = 1; // Reset but count this attempt
      canErrorMonitor.adaptiveMode = true;  // Enable adaptive recovery
    }
    else
    {
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
  if (canErrorMonitor.adaptiveMode)
  {
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
  if (err != mcp2515.ERROR_OK)
  {
    Serial.printlnf("CRITICAL: MCP2515 reset failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 reset failed");
    return;
  }

  delay(200);

  // Reconfigure controller
  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != mcp2515.ERROR_OK)
  {
    Serial.printlnf(
        "CRITICAL: MCP2515 bitrate config failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 config failed");
    return;
  }

  delay(100);

  err = mcp2515.setNormalMode();
  if (err != mcp2515.ERROR_OK)
  {
    Serial.printlnf(
        "CRITICAL: MCP2515 normal mode failed - forcing device reset");
    delay(100);
    resetDevice("MCP2515 mode failed");
    return;
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
      canErrorMonitor.extendedRecoveryDelay > 1000)
  {
    postRecoveryDelay = canErrorMonitor.extendedRecoveryDelay;
    Serial.printlnf("Extended post-recovery delay: %lu ms", postRecoveryDelay);
  }

  delay(postRecoveryDelay);
}

void logCANError(int errorCode, const char *operation)
{
  unsigned long currentTime = millis();

  canErrorMonitor.totalErrors++;
  canErrorMonitor.lastErrorTime = currentTime;
  canErrorMonitor.consecutiveErrors++;

  Serial.printlnf("CAN Error #%lu: %s failed with code %d (consecutive: %d)",
                  canErrorMonitor.totalErrors, operation, errorCode,
                  canErrorMonitor.consecutiveErrors);

  // Check for specific error codes that indicate controller corruption
  if (errorCode == 0xA || errorCode == 10 || errorCode == 0xF ||
      errorCode == 15)
  {
    Serial.printlnf(
        "CRITICAL: Detected ERROR_A/ERROR_F - MCP2515 controller corruption!");
    Serial.printlnf(
        "Triggering immediate CAN recovery to prevent system freeze");
    can_recovery_needed = true;

    // If we've seen these errors multiple times, force device reset
    if (canErrorMonitor.consecutiveErrors >= 2)
    {
      Serial.printlnf(
          "Multiple controller corruption errors - forcing device reset");
      delay(100);
      resetDevice("MCP2515 controller corruption");
      return;
    }
  }

  // Lower threshold for faster recovery during error cascades
  if (canErrorMonitor.consecutiveErrors >= 3)
  {
    Serial.printlnf("Fast recovery trigger - %d consecutive errors",
                    canErrorMonitor.consecutiveErrors);
    can_recovery_needed = true;
  }

  // Legacy compatibility - keep old variables for now
  can_error_count++;
  last_can_error_time = currentTime;
  if (can_error_count > MAX_CAN_ERRORS_PER_MINUTE)
  {
    can_recovery_needed = true;
  }
}

void resetCANSuccessCounter()
{
  // Reset consecutive error count on successful operation
  if (canErrorMonitor.consecutiveErrors > 0)
  {
    Serial.printlnf("CAN operation successful - resetting error count");
    canErrorMonitor.consecutiveErrors = 0;
  }

  // Update success time
  canErrorMonitor.lastSuccessTime = millis();

  // Reset recovery attempts after sustained successful operation
  if (canErrorMonitor.recoveryAttempts > 0)
  {
    unsigned long timeSinceRecovery =
        millis() - canErrorMonitor.lastRecoveryAttempt;

    // Only reset attempts if we've had sustained success (30 seconds)
    if (timeSinceRecovery > 30000)
    {
      Serial.printlnf(
          "Resetting recovery attempt counter after sustained success (%lu ms)",
          timeSinceRecovery);
      canErrorMonitor.recoveryAttempts = 0;
      canErrorMonitor.adaptiveMode =
          false; // Disable adaptive mode on sustained success
      canErrorMonitor.extendedRecoveryDelay = 0;
    }
  }

  // Only clear recovery mode if we're not currently in an active recovery
  if (!can_recovery_needed && !CAN_ERROR)
  {
    canErrorMonitor.inRecoveryMode = false;
  }
}

void checkCANHealth()
{
  unsigned long currentTime = millis();

  // Check if we have consecutive CAN errors with lower threshold
  if (canErrorMonitor.consecutiveErrors >= 3)
  {
    if (!canErrorMonitor.inRecoveryMode)
    {
      Serial.printlnf("CAN Error Threshold Reached: %d consecutive errors",
                      canErrorMonitor.consecutiveErrors);
      can_recovery_needed = true;
      canErrorMonitor.inRecoveryMode = true;
      canErrorMonitor.lastRecoveryAttempt = currentTime;
    }
  }

  // Reset consecutive error count if enough time has passed without errors
  if (currentTime - canErrorMonitor.lastErrorTime > CAN_ERROR_WINDOW)
  {
    if (canErrorMonitor.consecutiveErrors > 0)
    {
      Serial.printlnf(
          "CAN Error Window Expired - Resetting consecutive error count");
      canErrorMonitor.consecutiveErrors = 0;
    }
  }

  // Check if we've been in recovery mode too long
  if (canErrorMonitor.inRecoveryMode &&
      currentTime - canErrorMonitor.lastRecoveryAttempt >
          CAN_RECOVERY_DELAY * 2)
  {
    Serial.printlnf("CAN Recovery timeout - forcing device reset");
    delay(100);
    resetDevice("CAN recovery timeout");
  }
}

// ========================================
// Watchdog Functions
// ========================================

void enableWatchdog()
{
  watchdog_enabled = true;
  last_watchdog_feed = millis();
  Serial.printlnf("Watchdog enabled with %lu ms timeout", WATCHDOG_TIMEOUT);
}

void feedWatchdog()
{
  if (watchdog_enabled)
  {
    last_watchdog_feed = millis();
  }
}

void checkWatchdog()
{
  if (watchdog_enabled)
  {
    unsigned long currentTime = millis();
    if (currentTime - last_watchdog_feed > WATCHDOG_TIMEOUT)
    {
      Serial.printlnf(
          "Watchdog timeout! System has been unresponsive for %lu ms",
          currentTime - last_watchdog_feed);
      resetDevice("Watchdog timeout");
    }
  }
}

// ========================================
// Utility Functions
// ========================================

void reportCANError(int err, const char *operation, bool report)
{
  if (report)
  {
    CAN_ERROR = err != 0;
  }

  if (err != 0)
  {
    char ret[150];
    Serial.printlnf("CAN BUS ERROR %s", operation);
    ReturnErrorString(err, ret, sizeof(ret));
    sprintf(can_err_msg, "CAN BUS ERROR %s: %s", operation, ret);
    Serial.printlnf("%s", can_err_msg);

    // Log this as a CAN error for monitoring
    logCANError(err, operation);
  }
}

int resetDevice(String command)
{
  if (command.length() > 0)
  {
    Serial.printlnf("Device reset requested. Reason: %s", command.c_str());
    Serial.printlnf("Free memory before reset: %lu bytes", System.freeMemory());
    Serial.printlnf("System uptime: %lu ms", millis());
    Serial.printlnf("MQTT connected: %s", isMQTTConnected() ? "yes" : "no");
    Serial.printlnf("Credentials valid: %s",
                    areCredentialsValid() ? "yes" : "no");
    Serial.printlnf("CAN errors in last period: %d", can_error_count);

    if (isMQTTConnected())
    {
      Serial.printlnf("Closing MQTT connection...");
    }

    delay(1000);
  }

  System.reset();
  return 1;
}

void logDebugInfo(const char *checkpoint)
{
  static unsigned long lastLogTime = 0;
  unsigned long now = millis();

  Serial.printlnf("[%lu ms] Checkpoint: %s (Delta: %lu ms)", now, checkpoint,
                  now - lastLogTime);

  lastLogTime = now;
}

void logResetReason()
{
  int reason = System.resetReason();
  int reasonData = System.resetReasonData();

  Serial.printlnf("RESET REASON: %d", reason);
  Serial.printlnf("RESET REASON DATA: %d", reasonData);

  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Reason: %d, Data: %d", reason, reasonData);

  Particle.publish("RESET REASON", buffer, PRIVATE);
}

void checkSystemHealth()
{
  unsigned long freeMemory = System.freeMemory();
  unsigned long uptime = millis();

  if (freeMemory < 1000)
  {
    Serial.printlnf("Low memory warning: %lu bytes", freeMemory);
  }

  static unsigned long lastHealthCheck = 0;
  if (uptime - lastHealthCheck > 60000)
  {
    Serial.printlnf("System Health - Uptime: %lu ms, Free Memory: %lu bytes",
                    uptime, freeMemory);
    Serial.printlnf("MQTT Status: %s", getMQTTStatus().c_str());
    Serial.printlnf("Credentials: %s", getCredentialsStatus().c_str());
    Serial.printlnf("CAN Errors (last minute): %d", can_error_count);
    Serial.printlnf("CAN Recovery needed: %s",
                    can_recovery_needed ? "yes" : "no");

    if (portFlagHandler)
    {
      Serial.printlnf("Ports with pending flags: %d",
                      portFlagHandler->getPendingPortsCount());
    }

    lastHealthCheck = uptime;
  }
}

void emergencyReset(const char *reason)
{
  Serial.printlnf("EMERGENCY RESET: %s", reason);
  Serial.printlnf("Error Stats - Consecutive: %d, Total: %lu",
                  canErrorMonitor.consecutiveErrors,
                  canErrorMonitor.totalErrors);
  delay(200); // Ensure message is sent
  System.reset();
}

void canHealthMonitorThread()
{
  static unsigned long lastHealthCheck = 0;
  const unsigned long HEALTH_CHECK_INTERVAL =
      1000;                                      // Check every second during crisis
  const unsigned long MAIN_LOOP_TIMEOUT = 20000; // 20 seconds max

  Serial.printlnf("CAN Health Monitor thread started");

  while (true)
  {
    unsigned long currentTime = millis();

    // Check health every second
    if (currentTime - lastHealthCheck >= HEALTH_CHECK_INTERVAL)
    {

      // Emergency reset if error cascade continues
      if (canErrorMonitor.consecutiveErrors >= 10)
      {
        emergencyReset("Excessive consecutive CAN errors");
      }

      // Reset if stuck in recovery mode too long
      if (canErrorMonitor.inRecoveryMode &&
          currentTime - canErrorMonitor.lastRecoveryAttempt > 10000)
      {
        emergencyReset("CAN recovery timeout");
      }

      // Reset if no successful operations for too long
      if (canErrorMonitor.lastSuccessTime > 0 &&
          currentTime - canErrorMonitor.lastSuccessTime > 60000)
      { // 1 minute
        emergencyReset("No successful CAN operations for 60 seconds");
      }

      // Check for watchdog issues
      if (watchdog_enabled &&
          currentTime - last_watchdog_feed > MAIN_LOOP_TIMEOUT)
      {
        emergencyReset("Main loop freeze detected");
      }

      // Check CAN error rate with lower threshold
      if (canErrorMonitor.consecutiveErrors >= 6)
      {
        Serial.printlnf("CRITICAL: Excessive CAN errors (%d), forcing reset",
                        canErrorMonitor.consecutiveErrors);
        delay(100);
        emergencyReset("Excessive CAN errors");
      }

      // Check for queue overflow conditions
      if (queueOverflow)
      {
        Serial.printlnf("WARNING: CAN message queue overflow detected");
        queueOverflow = false; // Reset flag
        logCANError(-3, "persistent_queue_overflow");
      }

      // Check for invalid CAN messages (indicates controller corruption)
      if (canErrorMonitor.consecutiveErrors >= 3)
      {
        Serial.printlnf("WARNING: CAN controller may be corrupted (%d "
                        "consecutive errors), monitoring closely",
                        canErrorMonitor.consecutiveErrors);
      }

      // Log periodic health status during error conditions
      if (canErrorMonitor.consecutiveErrors > 0 ||
          canErrorMonitor.inRecoveryMode)
      {
        Serial.printlnf("CAN Health Status - Consecutive Errors: %d, Recovery "
                        "Attempts: %d, In Recovery: %s",
                        canErrorMonitor.consecutiveErrors,
                        canErrorMonitor.recoveryAttempts,
                        canErrorMonitor.inRecoveryMode ? "YES" : "NO");
      }

      lastHealthCheck = currentTime;
    }

    // Feed hardware watchdog during health monitoring
    ApplicationWatchdog::checkin();

    delay(1000); // Check every second
  }
}
