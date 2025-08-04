#ifndef CREDENTIALS_H
#define CREDENTIALS_H

#include "Arduino.h"
#include "Particle.h"

// Credentials State
extern char deviceIdBuf[100];
extern bool credentialsFetched;
extern int attemptedCredentialsFetchCount;
extern long last_credentials_call;

// Credentials Structure
struct IotCredentials {
  String pubId;
  String particleId;
  String hub_uuid;
};

extern struct IotCredentials iotCreds;

// Credentials Functions
void getIotCredentials(const char *event, const char *data);
void requestCredentials();
bool areCredentialsValid();
void resetCredentialsState();
bool shouldRetryCredentials();
String getCredentialsStatus();
void initializeCredentials();

#endif // CREDENTIALS_H
