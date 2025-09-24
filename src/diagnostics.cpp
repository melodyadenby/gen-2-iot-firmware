#include "diagnostics.h"
#include "Arduino.h"
#include "Particle.h"
#include "port_state.h"
#include "mqtt.h"
#include <string.h>

// Global diagnostics objects
DiagnosticData diagnostics;
DiagnosticData lastBootDiagnostics;

// Track last EEPROM save time to prevent excessive writes
static unsigned long lastEEPROMSaveTime = 0;
static bool diagnosticsInitialized = false;

// External variables from main.ino
extern struct CANErrorMonitor {
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
  int rxOverflowCount;
  unsigned long firstRxOverflowTime;
  unsigned long lastRxOverflowClear;
} canErrorMonitor;

extern int MQTT_FAIL_COUNT;
extern bool mqttHealthy;
extern bool CELLULAR_CONNECTED;
extern unsigned long lastInterruptTime;
extern bool interruptHealthy;
extern uint32_t total_messages_received;
extern volatile bool queueOverflow;
extern volatile int messageCount;
extern int attemptedCredentialsFetchCount;
extern uint8_t getCANErrorFlags(bool debugLog);

// Calculate checksum for diagnostic data
uint32_t calculateDiagnosticsChecksum(const DiagnosticData* data) {
    uint32_t checksum = 0;
    const uint8_t* bytes = (const uint8_t*)data;
    size_t length = sizeof(DiagnosticData) - sizeof(data->checksum);
    
    for (size_t i = 0; i < length; i++) {
        checksum += bytes[i];
        checksum = (checksum << 1) | (checksum >> 31); // Rotate left by 1
    }
    
    return checksum;
}

// Initialize diagnostics system
void initializeDiagnostics() {
    Serial.println("Initializing diagnostics system...");
    
    // Try to load diagnostics from EEPROM
    if (loadDiagnosticsFromEEPROM()) {
        // Copy to lastBootDiagnostics for comparison
        memcpy(&lastBootDiagnostics, &diagnostics, sizeof(DiagnosticData));
        
        // Print diagnostics from last session
        printRetainedDiagnostics();
        
        // Increment reset count
        incrementResetCount();
        
        // Clear current diagnostics for new session
        clearDiagnostics();
        
        // Preserve reset count
        diagnostics.resetCount = lastBootDiagnostics.resetCount;
    } else {
        Serial.println("No valid diagnostics found in EEPROM - starting fresh");
        clearDiagnostics();
        diagnostics.resetCount = 0;
    }
    
    // Initialize with current values
    diagnostics.magic = DIAGNOSTICS_MAGIC;
    diagnostics.version = DIAGNOSTICS_VERSION;
    diagnosticsInitialized = true;
    
    // Save initial state
    saveDiagnosticsToEEPROM();
}

// Update diagnostics with current system state
void updateDiagnostics() {
    if (!diagnosticsInitialized) return;
    
    // Update timestamps
    diagnostics.timestamp = millis();
    diagnostics.unixTime = Time.now();
    
    // System health
    diagnostics.freeMemory = System.freeMemory();
    diagnostics.uptime = millis();
    
    // Track main loop delays
    static unsigned long lastUpdateTime = 0;
    if (lastUpdateTime > 0) {
        unsigned long loopDelay = diagnostics.timestamp - lastUpdateTime;
        if (loopDelay > diagnostics.mainLoopMaxDelay) {
            diagnostics.mainLoopMaxDelay = loopDelay;
        }
    }
    lastUpdateTime = diagnostics.timestamp;
    
    // CAN health
    diagnostics.canTotalErrors = canErrorMonitor.totalErrors;
    diagnostics.canConsecutiveErrors = canErrorMonitor.consecutiveErrors;
    diagnostics.canRecoveryAttempts = canErrorMonitor.recoveryAttempts;
    diagnostics.canRecoveryNeeded = canErrorMonitor.inRecoveryMode;
    diagnostics.mcp2515Flags = getCANErrorFlags(false);
    diagnostics.rxOverflowCount = canErrorMonitor.rxOverflowCount;
    
    // Network health
    diagnostics.mqttFailCount = MQTT_FAIL_COUNT;
    diagnostics.mqttWasHealthy = mqttHealthy;
    diagnostics.cellularWasConnected = CELLULAR_CONNECTED;
    diagnostics.credentialAttempts = attemptedCredentialsFetchCount;
    
    if (isMQTTConnected()) {
        diagnostics.lastMqttConnected = millis();
    }
    
    // Get cellular signal strength
    CellularSignal sig = Cellular.RSSI();
    diagnostics.rssi = sig.getStrength();
    
    // Interrupt health
    diagnostics.timeSinceLastInterrupt = millis() - lastInterruptTime;
    diagnostics.interruptHealthy = interruptHealthy;
    
    // Message statistics
    diagnostics.totalMessagesReceived = total_messages_received;
    if (queueOverflow) {
        diagnostics.queueOverflowCount++;
    }
    if (messageCount > diagnostics.messageQueueMaxDepth) {
        diagnostics.messageQueueMaxDepth = messageCount;
    }
    
    // Update port communication statistics
    updatePortCommunicationStats();
    
    // Schedule periodic save to EEPROM (rate-limited)
    scheduleDiagnosticsSave();
}

// Update port communication statistics
void updatePortCommunicationStats() {
    int currentTime = Time.now();
    diagnostics.portsNotResponding = 0;
    diagnostics.longestPortSilenceTime = 0;
    diagnostics.portWithLongestSilence = 0;
    
    for (int i = 1; i <= MAX_PORTS; i++) {
        PortState *state = getPortState(i);
        if (state) {
            // Store last update time
            diagnostics.portLastUpdateTime[i-1] = state->last_msg_update_time;
            
            // Check for non-responsive ports
            if (state->last_msg_update_time > 0) {
                uint32_t silenceTime = currentTime - state->last_msg_update_time;
                
                if (silenceTime > PORT_SILENCE_WARNING_TIME) {
                    diagnostics.portsNotResponding++;
                }
                
                if (silenceTime > diagnostics.longestPortSilenceTime) {
                    diagnostics.longestPortSilenceTime = silenceTime;
                    diagnostics.portWithLongestSilence = i;
                }
            }
        }
    }
}

// Save diagnostics to EEPROM
void saveDiagnosticsToEEPROM() {
    // Calculate checksum
    diagnostics.checksum = calculateDiagnosticsChecksum(&diagnostics);
    
    // Write to EEPROM
    EEPROM.put(DIAGNOSTICS_EEPROM_ADDR, diagnostics);
    
    Serial.printlnf("Diagnostics saved to EEPROM (size: %d bytes)", sizeof(DiagnosticData));
    lastEEPROMSaveTime = millis();
}

// Load diagnostics from EEPROM
bool loadDiagnosticsFromEEPROM() {
    DiagnosticData tempData;
    
    // Read from EEPROM
    EEPROM.get(DIAGNOSTICS_EEPROM_ADDR, tempData);
    
    // Validate the data
    if (validateDiagnostics(&tempData)) {
        memcpy(&diagnostics, &tempData, sizeof(DiagnosticData));
        Serial.println("Valid diagnostics loaded from EEPROM");
        return true;
    }
    
    return false;
}

// Validate diagnostic data
bool validateDiagnostics(const DiagnosticData* data) {
    // Check magic number
    if (data->magic != DIAGNOSTICS_MAGIC) {
        Serial.printlnf("Invalid magic number: 0x%08X (expected: 0x%08X)", 
                       data->magic, DIAGNOSTICS_MAGIC);
        return false;
    }
    
    // Check version compatibility
    if (data->version != DIAGNOSTICS_VERSION) {
        Serial.printlnf("Version mismatch: %d (expected: %d)", 
                       data->version, DIAGNOSTICS_VERSION);
        return false;
    }
    
    // Verify checksum
    uint32_t calculatedChecksum = calculateDiagnosticsChecksum(data);
    if (data->checksum != calculatedChecksum) {
        Serial.printlnf("Checksum mismatch: 0x%08X (calculated: 0x%08X)", 
                       data->checksum, calculatedChecksum);
        return false;
    }
    
    // Basic sanity checks
    if (data->timestamp == 0 || data->uptime == 0) {
        Serial.println("Invalid timestamp or uptime");
        return false;
    }
    
    return true;
}

// Clear diagnostic data
void clearDiagnostics() {
    uint16_t savedResetCount = diagnostics.resetCount;
    memset(&diagnostics, 0, sizeof(DiagnosticData));
    diagnostics.magic = DIAGNOSTICS_MAGIC;
    diagnostics.version = DIAGNOSTICS_VERSION;
    diagnostics.resetCount = savedResetCount;
}

// Log diagnostics before reset
void logDiagnosticsBeforeReset(const char* reason) {
    // Update all diagnostics one final time
    updateDiagnostics();
    
    // Store reset reason
    diagnostics.resetReason = System.resetReason();
    diagnostics.resetReasonData = System.resetReasonData();
    diagnostics.previousUptime = millis();
    
    // Copy error message
    if (reason) {
        strncpy(diagnostics.lastError, reason, sizeof(diagnostics.lastError) - 1);
        diagnostics.lastError[sizeof(diagnostics.lastError) - 1] = '\0';
    }
    
    // Force save to EEPROM
    saveDiagnosticsToEEPROM();
    
    Serial.printlnf("Diagnostics saved before reset: %s", reason);
}

// Print retained diagnostics from previous session
void printRetainedDiagnostics() {
    Serial.println("=== DIAGNOSTICS FROM PREVIOUS SESSION ===");
    Serial.printlnf("Reset count: %d", diagnostics.resetCount);
    Serial.printlnf("Previous uptime: %lu ms (%lu minutes)", 
                   diagnostics.previousUptime, diagnostics.previousUptime / 60000);
    Serial.printlnf("Last save timestamp: %s", Time.format(diagnostics.unixTime, TIME_FORMAT_ISO8601_FULL).c_str());
    
    // System health
    Serial.println("--- System Health ---");
    Serial.printlnf("Free memory: %lu bytes", diagnostics.freeMemory);
    if (diagnostics.freeMemory < CRITICAL_FREE_MEMORY) {
        Serial.println("  ⚠️  CRITICAL: Low memory condition!");
    }
    Serial.printlnf("Main loop max delay: %lu ms", diagnostics.mainLoopMaxDelay);
    if (diagnostics.mainLoopMaxDelay > 5000) {
        Serial.println("  ⚠️  WARNING: Long main loop delays detected!");
    }
    
    // CAN health
    Serial.println("--- CAN Bus Health ---");
    Serial.printlnf("Total CAN errors: %lu", diagnostics.canTotalErrors);
    Serial.printlnf("Consecutive errors: %u", diagnostics.canConsecutiveErrors);
    Serial.printlnf("Recovery attempts: %u", diagnostics.canRecoveryAttempts);
    Serial.printlnf("Recovery needed: %s", diagnostics.canRecoveryNeeded ? "YES" : "NO");
    Serial.printlnf("MCP2515 flags: 0x%02X", diagnostics.mcp2515Flags);
    Serial.printlnf("RX overflow count: %u", diagnostics.rxOverflowCount);
    if (diagnostics.canTotalErrors > MAX_CAN_ERRORS_BEFORE_CONCERN) {
        Serial.println("  ⚠️  WARNING: High CAN error rate!");
    }
    
    // Network health
    Serial.println("--- Network Health ---");
    Serial.printlnf("MQTT fail count: %lu", diagnostics.mqttFailCount);
    Serial.printlnf("MQTT was healthy: %s", diagnostics.mqttWasHealthy ? "YES" : "NO");
    Serial.printlnf("Cellular was connected: %s", diagnostics.cellularWasConnected ? "YES" : "NO");
    Serial.printlnf("Cellular RSSI: %d dBm", diagnostics.rssi);
    Serial.printlnf("Credential attempts: %u", diagnostics.credentialAttempts);
    if (diagnostics.mqttFailCount > MAX_MQTT_FAILURES) {
        Serial.println("  ⚠️  WARNING: MQTT connection issues!");
    }
    
    // Interrupt health
    Serial.println("--- Interrupt Health ---");
    Serial.printlnf("Time since last interrupt: %lu ms", diagnostics.timeSinceLastInterrupt);
    Serial.printlnf("Interrupt healthy: %s", diagnostics.interruptHealthy ? "YES" : "NO");
    if (!diagnostics.interruptHealthy) {
        Serial.println("  ⚠️  CRITICAL: Interrupt system failure!");
    }
    
    // Message statistics
    Serial.println("--- Message Statistics ---");
    Serial.printlnf("Total messages received: %lu", diagnostics.totalMessagesReceived);
    Serial.printlnf("Queue overflow count: %lu", diagnostics.queueOverflowCount);
    Serial.printlnf("Max queue depth: %lu", diagnostics.messageQueueMaxDepth);
    
    // Port health
    Serial.println("--- Port Communication Health ---");
    Serial.printlnf("Non-responsive ports: %u", diagnostics.portsNotResponding);
    if (diagnostics.portWithLongestSilence > 0) {
        Serial.printlnf("Port %u has been silent for %lu seconds", 
                       diagnostics.portWithLongestSilence, 
                       diagnostics.longestPortSilenceTime);
    }
    
    // Print individual port status
    int currentTime = Time.now();
    for (int i = 1; i <= MAX_PORTS; i++) {
        int lastUpdate = diagnostics.portLastUpdateTime[i-1];
        if (lastUpdate > 0) {
            int timeSince = currentTime - lastUpdate;
            if (timeSince > PORT_SILENCE_CRITICAL_TIME) {
                Serial.printlnf("  Port %d: CRITICAL - No messages for %d seconds!", i, timeSince);
            } else if (timeSince > PORT_SILENCE_WARNING_TIME) {
                Serial.printlnf("  Port %d: WARNING - No messages for %d seconds", i, timeSince);
            }
        }
    }
    
    // Reset information
    Serial.println("--- Reset Information ---");
    Serial.printlnf("Reset reason: %s (%u)", getResetReasonString(diagnostics.resetReason), diagnostics.resetReason);
    Serial.printlnf("Reset reason data: %u", diagnostics.resetReasonData);
    Serial.printlnf("Last error: %s", diagnostics.lastError);
    
    Serial.println("=== END DIAGNOSTICS ===\n");
    
    // Publish summary to cloud
    publishDiagnosticsToCloud();
}

// Get reset reason as string
const char* getResetReasonString(uint8_t reason) {
    switch(reason) {
        case RESET_REASON_NONE: return "None";
        case RESET_REASON_UNKNOWN: return "Unknown";
        case RESET_REASON_PIN_RESET: return "Pin Reset";
        case RESET_REASON_POWER_MANAGEMENT: return "Power Management";
        case RESET_REASON_POWER_DOWN: return "Power Down";
        case RESET_REASON_POWER_BROWNOUT: return "Brownout";
        case RESET_REASON_WATCHDOG: return "Watchdog";
        case RESET_REASON_UPDATE: return "Firmware Update";
        case RESET_REASON_UPDATE_TIMEOUT: return "Update Timeout";
        case RESET_REASON_FACTORY_RESET: return "Factory Reset";
        case RESET_REASON_SAFE_MODE: return "Safe Mode";
        case RESET_REASON_DFU_MODE: return "DFU Mode";
        case RESET_REASON_PANIC: return "Panic";
        case RESET_REASON_USER: return "User Reset";
        default: return "Unknown";
    }
}

// Schedule diagnostics save (rate-limited to prevent EEPROM wear)
void scheduleDiagnosticsSave() {
    unsigned long currentTime = millis();
    
    // Only save if enough time has passed since last save
    if (currentTime - lastEEPROMSaveTime >= DIAGNOSTIC_SAVE_INTERVAL) {
        saveDiagnosticsToEEPROM();
    }
}

// Increment reset count
void incrementResetCount() {
    diagnostics.resetCount++;
    Serial.printlnf("Device has been reset %u times", diagnostics.resetCount);
}

// Check for critical issues
bool hasPortCommunicationIssues() {
    return diagnostics.portsNotResponding > 0 || 
           diagnostics.longestPortSilenceTime > PORT_SILENCE_CRITICAL_TIME;
}

bool hasMemoryPressure() {
    return diagnostics.freeMemory < CRITICAL_FREE_MEMORY;
}

bool hasCANBusIssues() {
    return diagnostics.canTotalErrors > MAX_CAN_ERRORS_BEFORE_CONCERN ||
           diagnostics.canRecoveryNeeded ||
           diagnostics.rxOverflowCount > 5;
}

bool hasNetworkIssues() {
    return diagnostics.mqttFailCount > MAX_MQTT_FAILURES ||
           !diagnostics.cellularWasConnected ||
           diagnostics.rssi < -100;
}

// Get diagnostic summary
String getDiagnosticSummary() {
    String summary = "Diagnostics: ";
    
    if (hasMemoryPressure()) summary += "LOW_MEM ";
    if (hasCANBusIssues()) summary += "CAN_ERR ";
    if (hasNetworkIssues()) summary += "NET_ERR ";
    if (hasPortCommunicationIssues()) summary += "PORT_ERR ";
    if (!diagnostics.interruptHealthy) summary += "INT_ERR ";
    
    if (summary == "Diagnostics: ") {
        summary += "OK";
    }
    
    return summary;
}

// Publish diagnostics to cloud
void publishDiagnosticsToCloud() {
    char summary[256];
    snprintf(summary, sizeof(summary), 
             "Resets:%u,Reason:%s,CAN:%lu,MQTT:%lu,Mem:%lu,Ports:%u,Error:%s",
             diagnostics.resetCount,
             getResetReasonString(diagnostics.resetReason),
             diagnostics.canTotalErrors,
             diagnostics.mqttFailCount,
             diagnostics.freeMemory,
             diagnostics.portsNotResponding,
             diagnostics.lastError);
    
    Particle.publish("diagnostic_report", summary, PRIVATE);
    
    // Also publish critical alerts if needed
    if (hasMemoryPressure()) {
        Particle.publish("critical_alert", "LOW_MEMORY", PRIVATE);
    }
    if (hasCANBusIssues()) {
        Particle.publish("critical_alert", "CAN_BUS_FAILURE", PRIVATE);
    }
    if (hasPortCommunicationIssues()) {
        char portAlert[64];
        snprintf(portAlert, sizeof(portAlert), "PORT_%u_SILENT_%lu_SEC", 
                diagnostics.portWithLongestSilence, 
                diagnostics.longestPortSilenceTime);
        Particle.publish("critical_alert", portAlert, PRIVATE);
    }
}

// Check if EEPROM contains valid diagnostics
bool isEEPROMDiagnosticsValid() {
    DiagnosticData tempData;
    EEPROM.get(DIAGNOSTICS_EEPROM_ADDR, tempData);
    return validateDiagnostics(&tempData);
}

// Erase diagnostics from EEPROM
void eraseDiagnosticsFromEEPROM() {
    DiagnosticData emptyData;
    memset(&emptyData, 0xFF, sizeof(DiagnosticData));
    EEPROM.put(DIAGNOSTICS_EEPROM_ADDR, emptyData);
    Serial.println("Diagnostics erased from EEPROM");
}

// Get size of diagnostics structure
size_t getDiagnosticsSize() {
    return sizeof(DiagnosticData);
}