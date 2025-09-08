#ifndef MAIN_H
#define MAIN_H

// System Configuration - must be first
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler(LOG_LEVEL_INFO);

retained uint8_t retainedLogs[2048];

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
#define CAN_QUEUE_SIZE 100           // Increased buffer size to handle poor network conditions
#define CAN_MAX_CONSECUTIVE_ERRORS 3 // Number of errors before controller reset

// CAN message queue variables (defined in main.ino)
extern volatile bool queueOverflow;
extern volatile int messageCount;
extern volatile int queueHead;
extern volatile int queueTail;
extern can_frame messageQueue[CAN_QUEUE_SIZE];

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

// System State Management
void initializeSystem();
void handleSystemLoop();
void checkSystemHealth();
void initializeHardware();
void initializeParticle();
void handleCredentials();
void updateSystemStatus();
void logResetReason();

// CAN Error Monitoring and Recovery
void performCANRecovery();
void logCANError(int errorCode, const char *operation);

// Hardware Watchdog (ApplicationWatchdog) is used instead of software watchdog

#endif // MAIN_H
