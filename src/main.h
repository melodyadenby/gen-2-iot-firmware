#ifndef MAIN_H
#define MAIN_H

// System Configuration - must be first
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(AUTOMATIC);

// Basic includes only to avoid circular dependencies
#include "Arduino.h"
#include "Particle.h"

// Forward declarations for types from other modules
struct PortState;
struct IotCredentials;

// CAN Bus State - declared here to avoid circular includes
#define CAN_INT A4
extern char can_err_msg[200];
extern bool CAN_ERROR;
#define CAN_QUEUE_SIZE 64            // Increased buffer size to reduce overflow chance
#define CAN_RECOVERY_DELAY 50        // ms to delay after CAN error
#define CAN_MAX_CONSECUTIVE_ERRORS 3 // Number of errors before controller reset

can_frame canMessageQueue[CAN_QUEUE_SIZE]; // Circular buffer for CAN messages
volatile int queueHead = 0;
volatile int queueTail = 0;
volatile int messageCount = 0;
volatile bool queueOverflow = false; // Flag to track queue overflow

// Core System Functions
void setup();
void loop();
void can_interrupt();
void reportCANError(int err, const char *operation, bool report);
int resetDevice(String command);
void logDebugInfo(const char *checkpoint);
void receiveMessage(can_frame recMsg);

// System State Management
void initializeSystem();
void handleSystemLoop();
void checkSystemHealth();
void initializeHardware();
void initializeParticle();
void handleCredentials();
void updateSystemStatus();
void logResetReason();

#endif // MAIN_H
