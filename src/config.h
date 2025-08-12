#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"
#include "Particle.h"

// Version and Device Info
#define BUILD_VERSION "Gen 1.0.2"

// Time Conversion Constants
const int MIN_TO_MS_MULTIPLIER = 60000;
const int SEC_TO_MS_MULTIPLIER = 1000;

// Environment Definitions
#define ENV_PROD 0
#define ENV_QA 1
#define ENV_LOCAL 2

// Set the current environment
#define ENV 0

// Environment-specific Configuration
#if ENV == ENV_QA
#define MQTT_URL "qa-hub-mqtt-broker-config.kuhmute.net"
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/qa-credentials"
#define PARTICLE_CREDENTIALS "qa-credentials"
#elif ENV == ENV_PROD
#define MQTT_URL "hub-mqtt-broker-config.kuhmute.net"
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/credentials"
#define PARTICLE_CREDENTIALS "credentials"
#elif ENV == ENV_LOCAL
#define MQTT_URL "mqtt://198.111.63.104:1883"
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/qa-credentials"
#define PARTICLE_CREDENTIALS "qa-credentials"
#else
#define MQTT_URL "dev-hub-mqtt-broker-config.kuhmute.net"
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/credentials"
#define PARTICLE_CREDENTIALS "credentials"
#endif

// System Configuration
#define PRODUCT_VERSION_NUM 39
#define MAX_PORTS 16

// Timeout Configuration
#define MQTT_DISCONNECTED_TIMEOUT_SEC 5
#define CREDENTIALS_RETRY_INTERVAL_SEC 10
#define MAX_CREDENTIAL_ATTEMPTS 10
#define PARTICLE_KEEPALIVE_MIN 23
#define WATCHDOG_TIMEOUT_MS 60000

// Configuration Functions
const char *getCurrentEnvironment();

#endif // CONFIG_H
