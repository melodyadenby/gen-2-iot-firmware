#include "Arduino.h"
#include "Particle.h"
#include "fixes/json_compat.h"
#include <ArduinoJson.h>

#include "config.h"
#include "credentials.h"
#include "logging.h"
#include "cloud.h"
// Global credential variables
char deviceIdBuf[100];
bool credentialsFetched = false;
int attemptedCredentialsFetchCount = 0;
long last_credentials_call = 0;
struct IotCredentials iotCreds;

void initializeCredentials() {
  // Initialize device ID buffer
  String deviceId = Particle.deviceID();
  deviceId.toCharArray(deviceIdBuf, sizeof(deviceIdBuf));

  // Reset credentials state
  resetCredentialsState();

  // Subscribe to credentials response
  Particle.subscribe(PARTICLE_CREDENTIALS_SUBSCRIBE, getIotCredentials,
                     MY_DEVICES);
}

void getIotCredentials(const char *event, const char *data) {
  Log.info("Received response: %s", data);

  if (strlen(data) == 0) {
    Log.info("Empty credentials response");
    return;
  }

  // Parse JSON response
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, data);

  if (error) {
    Log.error("Failed to parse credentials JSON: %s", error.c_str());
    return;
  }

  // Extract credentials
  if (doc.containsKey("pubId")) {
    iotCreds.pubId = doc["pubId"].as<String>();
  }

  if (doc.containsKey("particleId")) {
    iotCreds.particleId = doc["particleId"].as<String>();
  }

  if (doc.containsKey("hub_uuid")) {
    iotCreds.hub_uuid = doc["hub_uuid"].as<String>();
  }
  Log.info("Hub UUID: " + iotCreds.hub_uuid);
  Log.info("Particle ID: " + iotCreds.particleId);
  Log.info("Pub_id: " + iotCreds.pubId);

  // Validate required fields
  if (iotCreds.pubId.length() > 0 && iotCreds.particleId.length() > 0) {
    // Case-insensitive comparison of Particle IDs
    String cloudParticleId = iotCreds.particleId;
    String deviceParticleId = Particle.deviceID();
    cloudParticleId.toLowerCase();
    deviceParticleId.toLowerCase();

    if (cloudParticleId != deviceParticleId) {
      Log.info("Particle ID mismatch - Cloud: '%s', Device: '%s'",
                      iotCreds.particleId.c_str(), Particle.deviceID().c_str());
      credentialsFetched = false;
      return;
    }
    credentialsFetched = true;
    // Copy values to global variables
    strncpy(MANUAL_MODE, iotCreds.pubId, sizeof(MANUAL_MODE) - 1);
    MANUAL_MODE[sizeof(MANUAL_MODE) - 1] = '\0'; // Ensure null-terminated

    Log.info("Credentials successfully fetched and validated");
    Log.info("PubId: %s", iotCreds.pubId.c_str());
    Log.info("Hub UUID: %s", iotCreds.hub_uuid.c_str());
    Particle.unsubscribe();
    credentialsFetched = true;
  } else {
    Log.info("Invalid credentials received");
    credentialsFetched = false;
  }
}

void requestCredentials() {
  if (attemptedCredentialsFetchCount >= MAX_CREDENTIAL_ATTEMPTS) {
    Log.info("Max credential fetch attempts reached");
    return;
  }

  unsigned long currentTime = millis();
  if (currentTime - last_credentials_call <
      (CREDENTIALS_RETRY_INTERVAL_SEC * SEC_TO_MS_MULTIPLIER)) {
    // Not enough time has passed since last request
    return;
  }

  Log.info("Requesting credentials (attempt %d/%d)",
                  attemptedCredentialsFetchCount + 1, MAX_CREDENTIAL_ATTEMPTS);

  Particle.publish(PARTICLE_CREDENTIALS, deviceIdBuf, PRIVATE);
  attemptedCredentialsFetchCount++;
  last_credentials_call = currentTime;
}

bool areCredentialsValid() {
  return credentialsFetched && iotCreds.pubId.length() > 0 &&
         iotCreds.particleId.length() > 0;
}

void resetCredentialsState() {
  credentialsFetched = false;
  attemptedCredentialsFetchCount = 0;
  last_credentials_call = 0;
  iotCreds.pubId = "";
  iotCreds.particleId = "";
  iotCreds.hub_uuid = "";
}

bool shouldRetryCredentials() {
  if (credentialsFetched) {
    return false; // Already have valid credentials
  }

  if (attemptedCredentialsFetchCount >= MAX_CREDENTIAL_ATTEMPTS) {
    return false; // Max attempts reached
  }

  unsigned long currentTime = millis();
  return (currentTime - last_credentials_call) >=
         (CREDENTIALS_RETRY_INTERVAL_SEC * SEC_TO_MS_MULTIPLIER);
}

String getCredentialsStatus() {
  if (credentialsFetched) {
    return "Valid";
  } else if (attemptedCredentialsFetchCount >= MAX_CREDENTIAL_ATTEMPTS) {
    return "Failed - Max attempts reached";
  } else {
    return "Pending - Attempt " + String(attemptedCredentialsFetchCount) + "/" +
           String(MAX_CREDENTIAL_ATTEMPTS);
  }
}
