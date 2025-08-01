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

// Core System Functions
void setup();
void loop();
void can_interrupt();
void reportCANError(int err, const char *operation, bool report);
int resetDevice(String command);
void logDebugInfo(const char *checkpoint);

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
