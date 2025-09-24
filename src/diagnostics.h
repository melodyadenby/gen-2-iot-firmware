#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "config.h"
#include "Particle.h"

// EEPROM configuration
#define DIAGNOSTICS_EEPROM_ADDR 0        // Start address in EEPROM
#define DIAGNOSTICS_MAGIC 0xDEADBEEF     // Magic number to validate data
#define DIAGNOSTICS_VERSION 1             // Version for future compatibility

// Diagnostic structure that will be stored in EEPROM
struct DiagnosticData {
    // Validation fields
    uint32_t magic;           // Magic number to validate data integrity
    uint16_t version;         // Structure version
    uint32_t checksum;        // Checksum for data validation
    
    // Timestamp of last update
    uint32_t timestamp;       // millis() when diagnostics were saved
    int32_t unixTime;         // Unix timestamp for absolute time reference
    
    // CAN health metrics
    uint32_t canTotalErrors;
    uint8_t canConsecutiveErrors;
    uint8_t canLastErrorCode;
    uint8_t canRecoveryAttempts;
    bool canRecoveryNeeded;
    uint8_t mcp2515Flags;
    uint8_t rxOverflowCount;
    
    // System health metrics
    uint32_t freeMemory;
    uint32_t uptime;
    uint32_t mainLoopMaxDelay;
    
    // Network health metrics
    uint32_t mqttFailCount;
    uint32_t lastMqttConnected;
    bool mqttWasHealthy;
    bool cellularWasConnected;
    uint8_t credentialAttempts;
    
    // Interrupt health metrics
    uint32_t timeSinceLastInterrupt;
    bool interruptHealthy;
    uint32_t missedInterrupts;
    
    // Port health - last message timestamps for each port
    int32_t portLastUpdateTime[MAX_PORTS];  // Unix timestamps
    uint8_t portsNotResponding;             // Count of non-responsive ports
    uint8_t portWithLongestSilence;         // Which port has been silent longest
    uint32_t longestPortSilenceTime;        // How long that port has been silent
    
    // Queue and message statistics
    uint32_t totalMessagesReceived;
    uint32_t queueOverflowCount;
    uint32_t messageQueueMaxDepth;
    
    // Power and reset information
    uint8_t resetReason;         // System.resetReason()
    uint8_t resetReasonData;     // System.resetReasonData()
    uint32_t previousUptime;     // Uptime before last reset
    uint16_t resetCount;         // Number of resets (increments each boot)
    
    // Error tracking
    char lastError[48];          // Last error message before reset
    char lastCANOperation[16];   // Last CAN operation that failed
    
    // Environmental conditions (if you add sensors later)
    int8_t temperature;          // Temperature in Celsius (if available)
    uint8_t batteryLevel;        // Battery percentage (if available)
    int16_t rssi;               // Cellular signal strength
    
    // Padding for future expansion
    uint8_t reserved[32];        // Reserved for future use
};

// Global diagnostics object (not retained - will be loaded from EEPROM)
extern DiagnosticData diagnostics;
extern DiagnosticData lastBootDiagnostics;  // Copy of diagnostics from last boot

// Core diagnostic functions
void initializeDiagnostics();               // Call in setup() to load from EEPROM
void updateDiagnostics();                   // Update current diagnostics in RAM
void saveDiagnosticsToEEPROM();            // Save current diagnostics to EEPROM
void logDiagnosticsBeforeReset(const char* reason);  // Call before any reset
bool loadDiagnosticsFromEEPROM();          // Load diagnostics from EEPROM
bool validateDiagnostics(const DiagnosticData* data);  // Validate diagnostic data
void printRetainedDiagnostics();           // Print diagnostics from previous session
void clearDiagnostics();                   // Clear/reset diagnostic data
uint32_t calculateDiagnosticsChecksum(const DiagnosticData* data);  // Calculate checksum

// Diagnostic analysis functions
bool hasPortCommunicationIssues();         // Check if any ports are non-responsive
bool hasMemoryPressure();                  // Check if memory is critically low
bool hasCANBusIssues();                   // Check for CAN bus problems
bool hasNetworkIssues();                   // Check for network/MQTT issues
String getDiagnosticSummary();             // Get human-readable summary
void publishDiagnosticsToCloud();          // Publish diagnostics to Particle Cloud

// Utility functions
void incrementResetCount();                // Increment the reset counter
void updatePortCommunicationStats();       // Update port silence statistics
const char* getResetReasonString(uint8_t reason);  // Convert reset reason to string
void scheduleDiagnosticsSave();           // Schedule EEPROM write (rate-limited)

// EEPROM management
bool isEEPROMDiagnosticsValid();          // Check if EEPROM contains valid data
void eraseDiagnosticsFromEEPROM();        // Erase diagnostic data from EEPROM
size_t getDiagnosticsSize();              // Get size of diagnostics structure

// Diagnostic thresholds
#define CRITICAL_FREE_MEMORY 2048          // Bytes - below this is critical
#define MAX_CAN_ERRORS_BEFORE_CONCERN 10   // CAN errors before flagging issue
#define PORT_SILENCE_WARNING_TIME 300      // Seconds - warn if port silent this long
#define PORT_SILENCE_CRITICAL_TIME 600     // Seconds - critical if port silent this long
#define MAX_MQTT_FAILURES 5                // MQTT failures before concern
#define DIAGNOSTIC_SAVE_INTERVAL 60000     // Minimum ms between EEPROM writes

#endif // DIAGNOSTICS_H