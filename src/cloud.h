#ifndef CLOUD_H
#define CLOUD_H

#include "Arduino.h"
#include "Particle.h"

extern char MANUAL_MODE[14];
extern bool CELLULAR_CONNECTED;

extern char portStatusRequest[64];



int juiceMessageCallback(String payload);
int publishJuiseMessage(const char* message);
void processCloudCommand(char cmd, char variant, int port, char btn,
                        char *tokens[]);


// Port Status Request Functions
void sendPortStatus();
bool isPortStatusRequestPending();
void clearPortStatusRequest();
void checkPortStatusRequest();


#endif // CLOUD_H
