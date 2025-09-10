#ifndef CLOUD_H
#define CLOUD_H

#include "Arduino.h"
#include "Particle.h"

extern char MANUAL_MODE[14];
extern bool CELLULAR_CONNECTED;

extern char portStatusRequest[64];

// Message deduplication structure
struct MessageHistory {
    char lastMessage[128];
    unsigned long lastSentTime;
};

// Global message history for deduplication (MAX_PORTS + 1 for non-port messages at index 0)
extern struct MessageHistory messageHistory[];




int juiceMessageCallback(String payload);
int publishJuiseMessage(const char* message);

// Clear message history for deduplication (port 0 = clear all)
void clearMessageHistory(int port = 0);
void processCloudCommand(char cmd, char variant, int port, char btn,
                        char *tokens[]);


// Port Status Request Functions
void sendPortStatus();
bool isPortStatusRequestPending();
void clearPortStatusRequest();
void checkPortStatusRequest();


#endif // CLOUD_H
