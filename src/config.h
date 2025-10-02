#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"
#include "Particle.h"

// Version and Device Info
#define BUILD_VERSION "Gen 1.0.7"
#define PRODUCT_VERSION_NUM 45

// Time Conversion Constants
const int MIN_TO_MS_MULTIPLIER = 60000;
const int SEC_TO_MS_MULTIPLIER = 1000;
const int HOUR_TO_MS_MULTIPLIER = 60 * MIN_TO_MS_MULTIPLIER;

// Environment Definitions
#define ENV_PROD 0
#define ENV_QA 1
#define ENV_LOCAL 2

// Set the current environment
#define ENV ENV_LOCAL

// Environment-specific Configuration
#if ENV == ENV_QA
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/qa-credentials"
#define PARTICLE_CREDENTIALS "qa-credentials"
#define JUISE_INCOMING "juise-message"
#define JUISE_OUTGOING "juise-outgoing"
#elif ENV == ENV_PROD
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/credentials"
#define PARTICLE_CREDENTIALS "credentials"
#define JUISE_INCOMING "juise-message"
#define JUISE_OUTGOING "juise-outgoing"
#elif ENV == ENV_LOCAL
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/credentials"
#define PARTICLE_CREDENTIALS "credentials"
#define JUISE_INCOMING "juise-message"
#define JUISE_OUTGOING "local-juise-outgoing"
#else
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/credentials"
#define PARTICLE_CREDENTIALS "credentials"
#define JUISE_INCOMING "juise-message"
#define JUISE_OUTGOING "juise-outgoing"
#endif

// System Configuration
#define MAX_PORTS 16

// Timeout Configuration
#define MQTT_DISCONNECTED_TIMEOUT_SEC 5
#define CREDENTIALS_RETRY_INTERVAL_SEC 10
#define MAX_CREDENTIAL_ATTEMPTS 10
#define PARTICLE_KEEPALIVE_MIN 23
#define WATCHDOG_TIMEOUT_MS 60000

// Message Deduplication Configuration
#define MESSAGE_DEDUP_WINDOW_MS 2000  // 2 seconds by default

// Configuration Functions
const char *getCurrentEnvironment();

#endif // CONFIG_H
