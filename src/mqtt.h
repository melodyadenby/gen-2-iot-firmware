#ifndef MQTT_H
#define MQTT_H

#include "Arduino.h"
#include "Particle.h"
#include <MQTT.h>

// MQTT Configuration
#define MQTT_USR "sspitler"
#define MQTT_PWD "testpass"
#define MQTT_PORT 1883
#define KEEP_ALIVE 20 // seconds
#define MQTT_MAX_PACKET_SIZE 300

// MQTT Connection State
extern char MQTT_CLIENT_ID[45];
extern char MQTT_PUB_TOPIC[256];
extern char MQTT_SUB_TOPIC[256];
extern char MANUAL_MODE[14];
extern bool BROKER_CONNECTED;
extern bool CELLULAR_CONNECTED;
extern bool RESET_BROKER_FLAG;

extern char portStatusRequest[64];

// MQTT Retry Logic
extern unsigned long mqtt_disconnected_timer;
extern bool mqtt_disconnect_noted;
extern const unsigned long MQTT_DISCONNECTED_TIMEOUT;
extern const unsigned long RETRY_INTERVAL_MS;
extern unsigned long currentRetryInterval;
extern const unsigned long MAX_RETRY_INTERVAL_MS;
extern unsigned long lastRetryTime;
extern unsigned long lastConnected;
extern unsigned long last_mqtt_send;
extern int MQTT_FAIL_COUNT;
extern unsigned long lastHeartbeatRetryTime;

// MQTT Health Monitoring
extern unsigned long lastMqttMessageReceived;
extern bool mqttHealthy;

// MQTT Topics
extern char *topic_base;
extern char *SUB_BASE;
extern char *PUB_BASE;

// MQTT Client
extern MQTT client;

// MQTT Functions
void handleMQTTClientLoop();
void handleMQTT();
void checkMQTTStat();
void attemptReconnect();
void resetRetryLogic();
void increaseRetryInterval();
void build_topics(const char *base);
bool connect_mqtt();
bool connect_broker();
void sub_topic_and_alert();
void publishCloud(String message);
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void processMQTTCommand(char cmd, char variant, int port, char btn,
                        char *tokens[]);
void initializeMQTT();
bool isMQTTConnected();
String getMQTTStatus();
unsigned long getLastMQTTSend();
int getMQTTFailCount();
void resetMQTTFailCount();
bool shouldRetryMQTT();

// Port Status Request Functions
void sendPortStatus();
bool isPortStatusRequestPending();
void clearPortStatusRequest();

// MQTT Health and Heartbeat Functions
void checkMQTTHealth();
void sendHeartbeatIfNeeded(unsigned long currentTime);
bool isMQTTHealthy();
void forceMQTTReconnect();
void checkPortStatusRequest();

#endif // MQTT_H
