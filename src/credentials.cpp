#include "Arduino.h"
#include "Particle.h"
#include "fixes/json_compat.h"
#include <ArduinoJson.h>

#include "config.h"
#include "credentials.h"
#include "logging.h"
#include "mqtt.h"
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
  Serial.printlnf("Received response: %s", data);

  if (strlen(data) == 0) {
    Serial.println("Empty credentials response");
    return;
  }

  // Parse JSON response
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, data);

  if (error) {
    Serial.printlnf("Failed to parse credentials JSON: %s", error.c_str());
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
  Serial.println("Hub UUID: " + iotCreds.hub_uuid);
  Serial.println("Particle ID: " + iotCreds.particleId);
  Serial.println("Pub_id: " + iotCreds.pubId);

  // Validate required fields
  if (iotCreds.pubId.length() > 0 && iotCreds.particleId.length() > 0) {
    // Case-insensitive comparison of Particle IDs
    String cloudParticleId = iotCreds.particleId;
    String deviceParticleId = Particle.deviceID();
    cloudParticleId.toLowerCase();
    deviceParticleId.toLowerCase();

    if (cloudParticleId != deviceParticleId) {
      Serial.printlnf("Particle ID mismatch - Cloud: '%s', Device: '%s'",
                      iotCreds.particleId.c_str(), Particle.deviceID().c_str());
      credentialsFetched = false;
      return;
    }
    credentialsFetched = true;
    // Copy values to global variables
    strncpy(MANUAL_MODE, iotCreds.pubId, sizeof(MANUAL_MODE) - 1);
    MANUAL_MODE[sizeof(MANUAL_MODE) - 1] = '\0'; // Ensure null-terminated
    strncpy(MQTT_CLIENT_ID, Particle.deviceID(), sizeof(MQTT_CLIENT_ID) - 1);
    MQTT_CLIENT_ID[sizeof(MQTT_CLIENT_ID) - 1] = '\0'; // Ensure null-terminated

    Serial.printlnf("Credentials successfully fetched and validated");
    Serial.printlnf("PubId: %s", iotCreds.pubId.c_str());
    Serial.printlnf("MQTT User: %s", iotCreds.particleId.c_str());
    Serial.printlnf("Hub UUID: %s", iotCreds.hub_uuid.c_str());
    Particle.unsubscribe();
    build_topics(MQTT_USR);
    credentialsFetched = true;
    if (!connect_mqtt()) {
      Serial.println("Error: connect_mqtt");
    }
  } else {
    Serial.println("Invalid credentials received");
    credentialsFetched = false;
  }
}

void requestCredentials() {
  if (attemptedCredentialsFetchCount >= MAX_CREDENTIAL_ATTEMPTS) {
    Serial.println("Max credential fetch attempts reached");
    return;
  }

  unsigned long currentTime = millis();
  if (currentTime - last_credentials_call <
      (CREDENTIALS_RETRY_INTERVAL_SEC * SEC_TO_MS_MULTIPLIER)) {
    // Not enough time has passed since last request
    return;
  }

  Serial.printlnf("Requesting credentials (attempt %d/%d)",
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
