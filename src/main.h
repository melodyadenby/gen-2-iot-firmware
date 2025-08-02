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
class CANMessageProcessor;
class PortEventHandler;
class PortFlagHandler;

// CAN Bus State - declared here to avoid circular includes
#define CAN_INT A4
extern char can_err_msg[200];
extern bool CAN_ERROR;
#define CAN_QUEUE_SIZE 64            // Increased buffer size to reduce overflow chance
#define CAN_RECOVERY_DELAY 50        // ms to delay after CAN error
#define CAN_MAX_CONSECUTIVE_ERRORS 3 // Number of errors before controller reset

// CAN message queue variables (defined in main.ino)
extern volatile bool queueOverflow;
extern volatile int messageCount;
extern volatile int queueHead;
extern volatile int queueTail;
extern can_frame messageQueue[50];

// Architecture Components
extern PortEventHandler *portEventHandler;
extern PortFlagHandler *portFlagHandler;

// Core System Functions
void setup();
void loop();
void can_interrupt();
void reportCANError(int err, const char *operation, bool report);
int resetDevice(String command);
void logDebugInfo(const char *checkpoint);

// New Architecture Functions
void initializeArchitecture();
void canThread();
void handleCanQueue();
void processCANMessage(const can_frame &rawMessage);

// Legacy Functions (for compatibility)
void receiveMessage(can_frame recMsg);
int portWriteNew(int port, char cmd, char *variant, int timeout);
int portWriteParams(int port, char volts[], char amps[], int timeout);

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
