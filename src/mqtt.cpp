#include "mqtt.h"
#include "Arduino.h"
#include "Particle.h"
#include "config.h"
#include "credentials.h"
#include "fixes/json_compat.h"
#include "lights.h"
#include "logging.h"
#include "port_state.h"
#include "utils.h"
#include <ArduinoJson.h>

// Global MQTT variables
char MQTT_CLIENT_ID[45];
char MQTT_PUB_TOPIC[256];
char MQTT_SUB_TOPIC[256];
char MANUAL_MODE[14];
bool BROKER_CONNECTED = false;

// MQTT Retry Logic Variables
unsigned long mqtt_disconnected_timer = 0;
bool mqtt_disconnect_noted = false;
const unsigned long MQTT_DISCONNECTED_TIMEOUT =
    MQTT_DISCONNECTED_TIMEOUT_SEC * SEC_TO_MS_MULTIPLIER;
const unsigned long RETRY_INTERVAL_MS = SEC_TO_MS_MULTIPLIER;
unsigned long currentRetryInterval = RETRY_INTERVAL_MS;
const unsigned long MAX_RETRY_INTERVAL_MS = 15 * SEC_TO_MS_MULTIPLIER;
unsigned long lastRetryTime = 0;
unsigned long lastConnected = 0;
unsigned long last_mqtt_send = 0;
int MQTT_FAIL_COUNT = 0;
unsigned long lastHeartbeatRetryTime = 0;

// MQTT Health Monitoring Variables
unsigned long lastMqttMessageReceived = 0;
unsigned long lastMqttHealthCheck = 0;
const unsigned long MQTT_MESSAGE_TIMEOUT =
    300000; // 5 minutes without messages = unhealthy
const unsigned long MQTT_HEALTH_CHECK_INTERVAL =
    60000; // Check every 60 seconds
bool mqttHealthy = true;

// MQTT Heartbeat Variables
const unsigned long MQTT_HEARTBEAT_INTERVAL = 60 * 60 * 1000; // 1 hour
const unsigned long MQTT_HEARTBEAT_RETRY_INTERVAL = 5 * 1000; // 5 seconds
const int MQTT_HEARTBEAT_MAX_FAILURES = 5;

// Port status request variables
char portStatusRequest[64];
bool portStatusRequestPending = false;
unsigned long portStatusRequestTime = 0;
bool portStatusWaitingForPoll = false;

// MQTT Topics
char *topic_base = "/hub/";
char *SUB_BASE = "/cmd";
char *PUB_BASE = "/msg";

// MQTT Client
MQTT client(MQTT_URL, MQTT_PORT, MQTT_MAX_PACKET_SIZE, KEEP_ALIVE,
            mqtt_callback);

void initializeMQTT() {
  // Initialize MQTT state
  memset(MQTT_CLIENT_ID, 0, sizeof(MQTT_CLIENT_ID));
  memset(MQTT_PUB_TOPIC, 0, sizeof(MQTT_PUB_TOPIC));
  memset(MQTT_SUB_TOPIC, 0, sizeof(MQTT_SUB_TOPIC));
  memset(MANUAL_MODE, 0, sizeof(MANUAL_MODE));

  BROKER_CONNECTED = false;
  mqtt_disconnect_noted = false;
  resetRetryLogic();

  Serial.println("MQTT initialized");
}

void handleMQTTClientLoop() { client.loop(); }

// Forward declaration of handleRefreshedPortStates
void handleRefreshedPortStates();

// External function for sending port commands - defined in port_flag_handler.cpp
extern int sendPortCommand(int port, char command, const char *variant, int timeout);

void handleMQTT() {
  BROKER_CONNECTED = client.isConnected();
  checkMQTTStat();
  checkMQTTHealth();
  checkPortStatusRequest();
  handleRefreshedPortStates(); // Handle refreshed port states
}

void checkMQTTStat() {
  if (!areCredentialsValid()) {
    // Don't attempt reconnection until credentials are fetched
    return;
  }

  if (BROKER_CONNECTED) {
    resetRetryLogic();
    return;
  }

  unsigned long current_time = millis();
  // MQTT is disconnected
  if (!mqtt_disconnect_noted) {
    mqtt_disconnected_timer = millis();
    mqtt_disconnect_noted = true;
    Serial.println("MQTT disconnected - starting retry timer");
  }

  // Attempt reconnection after timeout, but respect retry intervals
  if (current_time - mqtt_disconnected_timer >= MQTT_DISCONNECTED_TIMEOUT) {
    // Only call attemptReconnect if enough time has passed since last attempt
    if (current_time - lastRetryTime >= currentRetryInterval) {
      attemptReconnect();
    }
  }
}

void checkMQTTHealth() {
  unsigned long currentTime = millis();

  // Only check health periodically
  if (currentTime - lastMqttHealthCheck < MQTT_HEALTH_CHECK_INTERVAL) {
    return;
  }
  lastMqttHealthCheck = currentTime;

  // Check if we haven't received messages in too long
  if (BROKER_CONNECTED && lastMqttMessageReceived > 0) {
    unsigned long timeSinceLastMessage = currentTime - lastMqttMessageReceived;
    if (timeSinceLastMessage > MQTT_MESSAGE_TIMEOUT) {
      Serial.printlnf("MQTT unhealthy - no messages for %lu ms",
                      timeSinceLastMessage);
      mqttHealthy = false;
      // Force reconnection
      client.disconnect();
      BROKER_CONNECTED = false;
      mqtt_disconnect_noted = false;
    }
  }

  // Send heartbeat if needed
  sendHeartbeatIfNeeded(currentTime);
}

void attemptReconnect() {
  unsigned long currentMillis = millis();

  // This function should only be called when ready to attempt reconnection
  Serial.printlnf("Time since last attempt: %lu ms",
                  currentMillis - lastRetryTime);
  Serial.printlnf("Current retry interval: %lu ms", currentRetryInterval);
  Serial.println("Attempting to reconnect MQTT...");

  if (connect_mqtt()) {
    Serial.println("Reconnected to MQTT broker.");
    resetRetryLogic();
  } else {
    Serial.println("Reconnect attempt failed.");
    lastRetryTime = currentMillis; // Update last retry time on attempt
    increaseRetryInterval();
  }
}

void resetRetryLogic() {
  mqtt_disconnect_noted = false;
  currentRetryInterval = RETRY_INTERVAL_MS; // Reset to initial retry interval
  lastRetryTime = millis();                 // Reset the timer to current time
}

void increaseRetryInterval() {
  currentRetryInterval *= 2; // Exponential backoff
  if (currentRetryInterval > MAX_RETRY_INTERVAL_MS) {
    currentRetryInterval = MAX_RETRY_INTERVAL_MS;
  }
}

void build_topics(const char *base) {
  if (MANUAL_MODE[0] != '\0') {
    snprintf(MQTT_PUB_TOPIC, sizeof(MQTT_PUB_TOPIC), "%s%s%s", topic_base,
             MANUAL_MODE, PUB_BASE);
    snprintf(MQTT_SUB_TOPIC, sizeof(MQTT_SUB_TOPIC), "%s%s%s", topic_base,
             MANUAL_MODE, SUB_BASE);
  } else {
    snprintf(MQTT_PUB_TOPIC, sizeof(MQTT_PUB_TOPIC), "%s%s%s", topic_base, base,
             PUB_BASE);
    snprintf(MQTT_SUB_TOPIC, sizeof(MQTT_SUB_TOPIC), "%s%s%s", topic_base, base,
             SUB_BASE);
  }

  Serial.printlnf("Pub Topic: %s\n", MQTT_PUB_TOPIC);
  Serial.printlnf("Sub Topic: %s\n", MQTT_SUB_TOPIC);
}

bool connect_mqtt() { return connect_broker(); }

bool connect_broker() {
  if (client.isConnected()) {
    Serial.println("Already connected to MQTT broker.");
    return true; // If already connected, don't attempt to reconnect
  }

  if (!Particle.connected()) {
    Serial.println("Internet not connected. Cannot connect to MQTT.");
    return false;
  }

  if (!areCredentialsValid()) {
    Serial.println("No valid credentials for MQTT connection.");
    return false;
  }

  Serial.println("Connecting to MQTT broker...");
  bool res = client.connect(MQTT_CLIENT_ID, MQTT_USR, MQTT_PWD, MQTT_PUB_TOPIC,
                            MQTT::QOS1, 5, "A,0,0", true);

  Serial.printlnf("MQTT Connection Status: %s\n", res ? "Connected" : "Failed");

  if (res) {
    if (lastConnected == 0 || lastConnected >= KEEP_ALIVE * 1000) {
      sub_topic_and_alert();
    }
    lastConnected = millis();
  } else {
    Serial.println("MQTT connection failed");
  }

  BROKER_CONNECTED = res;
  return res;
}

void sub_topic_and_alert() {
  Serial.printlnf("PUBLISHING A,0,1 to topic: %s\n", MQTT_PUB_TOPIC);
  int res = client.publish(MQTT_PUB_TOPIC, "A,0,1");
  Serial.printlnf("sub_topic_and_alert: MQTT PUB STAT: %d\n", res);

  if (!res) {
    Serial.println("FAILED TO PUBLISH");
    return;
  }

  Serial.printlnf("SUBSCRIBING to topic: %s\n", MQTT_SUB_TOPIC);
  res = client.subscribe(MQTT_SUB_TOPIC);
  Serial.printlnf("sub_topic_and_alert: MQTT SUB STAT: %d\n", res);

  if (!res) {
    Serial.println("FAILED TO SUBSCRIBE");
    return;
  }
}

void publishCloud(String message) {
  Serial.printlnf("Going to publish cloud: %s\n", message.c_str());
  Serial.printlnf("Publishing to topic: %s\n", MQTT_PUB_TOPIC);

  if (!BROKER_CONNECTED) {
    Serial.println("MQTT not connected, cannot publish");
    MQTT_FAIL_COUNT++;
    return;
  }

  // Check if client is actually connected (not just BROKER_CONNECTED flag)
  if (!client.isConnected()) {
    Serial.println("MQTT client not actually connected despite flag");
    BROKER_CONNECTED = false;
    mqtt_disconnect_noted = false;
    MQTT_FAIL_COUNT++;
    return;
  }

  bool res = client.publish(MQTT_PUB_TOPIC, message);
  Serial.printlnf("publishCloud result: %s\n", res ? "success" : "failed");

  if (res) {
    last_mqtt_send = millis(); // Update the last successful send time
    MQTT_FAIL_COUNT = 0;       // Reset failure count after success
  } else {
    MQTT_FAIL_COUNT++;
    lastHeartbeatRetryTime = millis(); // Store last failed attempt time
    Serial.printlnf("MQTT publish failed, fail count: %d\n", MQTT_FAIL_COUNT);
  }
}

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = '\0';

  Serial.printlnf("MQTT message received on topic '%s': %s", topic, p);

  // Update last message received time for health monitoring
  lastMqttMessageReceived = millis();
  mqttHealthy = true;

  // Parse the message by comma delimiter
  char *token;
  char *rest = p;
  char *tokens[4] = {NULL}; // Store up to 4 tokens (cmd, variant, port, btn)
  int i = 0;

  // Split the string by commas
  while ((token = strtok_r(rest, ",", &rest)) && i < 4) {
    tokens[i++] = token;
  }

  // Extract command from first token
  char cmd = tokens[0] ? tokens[0][0] : '\0';

  // Extract variant from second token if available
  char variant = tokens[1] ? tokens[1][0] : '\0';

  // Extract port from third token if available
  int port = 0;
  if (tokens[2]) {
    port = atoi(tokens[2]);
    Serial.printlnf("Parsed port number: %d\n", port);
  }

  // Extract button state from fourth token if available
  char btn = tokens[3] ? (tokens[3][0] == '0' ? '1' : '0') : '2';

  // Process commands
  processMQTTCommand(cmd, variant, port, btn, tokens);
}

void processMQTTCommand(char cmd, char variant, int port, char btn,
                        char *tokens[]) {
  struct PortState *portState = getPortState(port);

  switch (cmd) {
  case 'C': // Charge command
    if (isValidPort(port) && portState) {
      portState->check_charge_status = true;
      portState->charge_varient = '2'; // variant;
      portState->send_charge_flag = true;
      portState->awaiting_cloud_vin_resp = false;
      Serial.printlnf("Charge command for port %d, variant %c\n", port,
                      variant);
    } else {
      Serial.printlnf("Invalid port for charge command: %d\n", port);
    }
    break;

  case 'U': // Unlock command
    if (isValidPort(port) && portState) {
      portState->send_unlock_flag = true;
      Serial.printlnf("Unlock command for port %d\n", port);
    } else {
      Serial.printlnf("Invalid port for unlock command: %d\n", port);
    }
    break;
  case 'R': // Refresh data (NON BLOCKING OPERATION, timeout: 20000ms)
  // 1. Send R command to ports
  // 2. Send VIN Request to ports IF portstate->docked
  // 3. Send states over MQTT sequentially for each port (if varient is 0) 
  //    - MQTT: R,<port>,<port_data: Tag_Read_Success Latch_Status Secured Charge_Status FATAL_NFC_ERROR>,<VIN>
  //      - Ex- R,16,11110,EB10084227000041
  //      - Length: 25-26
    switch (variant) {
    case '0':
      if (tokens[3] && tokens[3][0] == '0') {
        // Update entire station state, send states sequentially
        Serial.println("DEBUG: Starting full station refresh");
        
        // Set the refresh flags for all valid ports
        for (int i = 1; i <= MAX_PORTS; i++) {
          if (isValidPort(i)) {
            PortState *ps = getPortState(i);
            if (ps) {
              // Mark port for refresh
              ps->await_refresh_port_update = true;
              ps->refresh_request_time = millis();
              
              // Queue the R command to this port
              // Use sendPortCommand from existing code pattern for other flags
              sendPortCommand(i, 'R', nullptr, 10 * SEC_TO_MS_MULTIPLIER);
              
              // If port is docked, also request VIN
              if (ps->docked) {
                ps->await_refresh_port_vin = true;
                ps->vin_request_flag = true;
                ps->send_vin_request_timer = millis();
              }
              
              // First port should trigger MQTT updates for all
              if (i == 1) {
                ps->send_refresh_port_update = true;
              }
            }
          }
        }
      } else {
        // Port specific state update
        if (isValidPort(port) && portState) {
          // Mark port for refresh
          portState->await_refresh_port_update = true;
          portState->refresh_request_time = millis();
          
          // Queue the R command to this port
          sendPortCommand(port, 'R', nullptr, 10 * SEC_TO_MS_MULTIPLIER);
          
          // If port is docked, also request VIN
          if (portState->docked) {
            portState->await_refresh_port_vin = true;
            portState->vin_request_flag = true;
            portState->send_vin_request_timer = millis();
          }
          
          // Mark to send this port's update over MQTT when ready
          portState->send_refresh_port_update = true;
        } else {
          Serial.printlnf("Invalid port for port state update: %d\n", port);
        }
      }
      break;
    case 'H': // Heartbeat command
      switch (variant) {
      case '0':
        if (tokens[3] && tokens[3][0] == '0') {
          // IoT heartbeat
          publishCloud("H,0,1");
          Serial.println("IoT heartbeat response sent");
        } else {
          // Port heartbeat
          if (isValidPort(port) && portState) {
            portState->send_port_heartbeat = true;
            Serial.printlnf("Port heartbeat command for port %d\n", port);
          } else {
            Serial.printlnf("Invalid port for heartbeat: %d\n", port);
          }
        }
        break;
      case 'P': // P,0,<port_start>, <port_end>
        if (tokens[2] && tokens[3]) {
          snprintf(portStatusRequest, sizeof(portStatusRequest), "P,0,%s,%s",
                   tokens[2], tokens[3]);
          portStatusRequestPending = true;
          portStatusRequestTime = millis();
          portStatusWaitingForPoll = true;
          Serial.printlnf("Port status request stored: %s", portStatusRequest);
          Serial.printlnf("Will wait for fresh port data before responding");
          Serial.printlnf(
              "DEBUG: Wait time will be %lu ms (PORT_CHECK_INTERVAL=%lu ms)",
              PORT_CHECK_INTERVAL + 5000, PORT_CHECK_INTERVAL);
        } else {
          Serial.println("Invalid P command format - missing port range");
        }
        break;
      default:
        Serial.printlnf("Unknown heartbeat variant: %c\n", variant);
        break;
      }
      break;
    case 'K': // K_VIN Validation command
      switch (variant) {
      case '0': {
        Serial.printlnf("Emergency Exit flag enabled for port: %d", port);
        if (port >= 1 && port <= MAX_PORTS) {
          portState->emergency_exit_flag = true;
        } else {
          Serial.println(
              "Invalid port number for emergency exit (VIN Invalid)");
        }
        break;
      }
      case '1': {
        Serial.println("VIN Validated, charging allowed");
        portState->check_charge_status = true;
        portState->charge_varient = '2'; // variant;
        portState->send_charge_flag = true;
        portState->awaiting_cloud_vin_resp = false;
        // Serial.println("Printing each character with index:");
        // for (int i = 0; i < strlen(p); i++)
        // {
        //   Serial.printf("p[%d]: %c", i, p[i]);
        //   Serial.println("");
        // }
        // char volts[3]; // Buffer for voltage with space for null terminator
        // char amps[3];  // Buffer for amperage with space for null terminator

        // // Assuming 'p' is a properly null-terminated string coming as input
        // volts[0] = p[8];
        // volts[1] = p[9];
        // volts[2] = '\0'; // Null-terminate the string

        // amps[0] = p[11];
        // amps[1] = p[12];
        // amps[2] = '\0'; // Null-terminate the string

        // Serial.printf("Volts: %s, Amps: %s", volts, amps);
        // Serial.println();

        // portState[port].button_state = btn;
        // portState[port].send_button_state_flag = true;

        // strcpy(portState[port].volts, volts);
        // strcpy(portState[port].amps, amps);
        // portState[port].send_charging_params_flag = true;
        break;
      }
      default:
        Serial.printlnf("Unknown K_VIN variant: %c\n", variant);
        break;
      }
      break;

    case 'V': // Version command
      switch (variant) {
      case '0':
        send_iot_build_version_flag = true;
        Serial.println("IoT version request");
        break;
      case '1':
        if (isValidPort(port) && portState) {
          portState->send_port_build_version_flag = true;
          Serial.printlnf("Port version request for port %d\n", port);
        } else {
          Serial.printlnf("Invalid port for version request: %d\n", port);
        }
        break;
      default:
        Serial.printlnf("Unknown version variant: %c\n", variant);
        break;
      }
      break;

    case 'T': // Temperature request
      if (isValidPort(port) && portState) {
        portState->send_temp_req_flag = true;
        Serial.printlnf("Temperature request for port %d\n", port);
      } else {
        Serial.printlnf("Invalid port for temperature request: %d\n", port);
      }
      break;

    case 'E': // Emergency exit
      if (isValidPort(port) && portState) {
        portState->emergency_exit_flag = true;
        Serial.printlnf("Emergency exit for port %d\n", port);
      } else {
        Serial.printlnf("Invalid port for emergency exit: %d\n", port);
      }
      break;

    case 'P': // Port status request: P,0,<port_start>,<port_end>
      if (tokens[2] && tokens[3]) {
        snprintf(portStatusRequest, sizeof(portStatusRequest), "P,0,%s,%s",
                 tokens[2], tokens[3]);
        portStatusRequestPending = true;
        portStatusRequestTime = millis();
        portStatusWaitingForPoll = true;
        Serial.printlnf("Port status request stored: %s", portStatusRequest);
        Serial.printlnf("Will wait for fresh port data before responding");
        Serial.printlnf(
            "DEBUG: Wait time will be %lu ms (PORT_CHECK_INTERVAL=%lu ms)",
            PORT_CHECK_INTERVAL + 5000, PORT_CHECK_INTERVAL);
      } else {
        Serial.println("Invalid P command format - missing port range");
      }
      break;

    default:
      Serial.printlnf("Unknown MQTT command: %c\n", cmd);
      break;
    }
  }

  bool isMQTTConnected() { return BROKER_CONNECTED && client.isConnected(); }

  String getMQTTStatus() {
    if (isMQTTConnected()) {
      return "Connected";
    } else if (!areCredentialsValid()) {
      return "Waiting for credentials";
    } else if (!Particle.connected()) {
      return "No internet connection";
    } else {
      return "Disconnected - Retrying";
    }
  }

  unsigned long getLastMQTTSend() { return last_mqtt_send; }

  int getMQTTFailCount() { return MQTT_FAIL_COUNT; }

  void resetMQTTFailCount() { MQTT_FAIL_COUNT = 0; }

  bool shouldRetryMQTT() {
    if (isMQTTConnected()) {
      return false;
    }

    if (!areCredentialsValid() || !Particle.connected()) {
      return false;
    }

    unsigned long currentTime = millis();
    return (currentTime - lastRetryTime) >= currentRetryInterval;
  }

  void sendPortStatus() {
    Serial.println(
        "DEBUG: sendPortStatus() called - checking if this should happen");
    Serial.printlnf(
        "DEBUG: portStatusRequestPending=%s, portStatusWaitingForPoll=%s",
        portStatusRequestPending ? "true" : "false",
        portStatusWaitingForPoll ? "true" : "false");
    Serial.printlnf("DEBUG: Request time=%lu, Current time=%lu, Elapsed=%lu ms",
                    portStatusRequestTime, millis(),
                    millis() - portStatusRequestTime);

    // Check if this is being called too early
    if (portStatusWaitingForPoll && portStatusRequestTime > 0) {
      unsigned long elapsed = millis() - portStatusRequestTime;
      unsigned long requiredWait = PORT_CHECK_INTERVAL + 5000;
      if (elapsed < requiredWait) {
        Serial.printlnf("ERROR: sendPortStatus() called too early! Only %lu ms "
                        "elapsed, need %lu ms",
                        elapsed, requiredWait);
        return; // Don't send status yet
      }
    }

    Serial.println("sending port staus");
    char requestCopy[sizeof(portStatusRequest)];
    strncpy(requestCopy, portStatusRequest, sizeof(requestCopy));
    requestCopy[sizeof(requestCopy) - 1] = '\0';

    char *token = strtok(requestCopy, ",");

    int port_start = -1;
    int port_end = -1;

    // Skip "P" token
    token = strtok(NULL, ",");

    // Skip "0" token
    token = strtok(NULL, ",");

    // Get port_start
    if (token != NULL) {
      port_start = atoi(token);
    }

    // Get port_end
    token = strtok(NULL, ",");
    if (token != NULL) {
      port_end = atoi(token);
    }

    if (port_start != -1 && port_end != -1) {
      char *retI = getPortStatusRange(port_start, port_end);

      Serial.println("sending port status");
      Serial.print("Sending Buffer: ");
      Serial.println(retI);
      publishCloud(retI); // Assuming publishCloud is defined elsewhere
    }
  }

  bool isPortStatusRequestPending() { return portStatusRequestPending; }

  void clearPortStatusRequest() {
    portStatusRequestPending = false;
    portStatusWaitingForPoll = false;
    portStatusRequestTime = 0;
    memset(portStatusRequest, 0, sizeof(portStatusRequest));
  }

  void sendHeartbeatIfNeeded(unsigned long currentTime) {
    // Send heartbeat every hour, with retry logic for failures
    if (currentTime - last_mqtt_send >= MQTT_HEARTBEAT_INTERVAL) {
      Serial.println("Sending MQTT heartbeat check");

      if (MQTT_FAIL_COUNT < MQTT_HEARTBEAT_MAX_FAILURES) {
        if (currentTime - lastHeartbeatRetryTime >=
            MQTT_HEARTBEAT_RETRY_INTERVAL) {
          Serial.printlnf("Heartbeat attempt #%d", MQTT_FAIL_COUNT + 1);
          publishCloud("H,0,1");
          lastHeartbeatRetryTime = currentTime;
        }
      } else {
        Serial.printlnf("MQTT failed %d times. Forcing reconnection...",
                        MQTT_HEARTBEAT_MAX_FAILURES);
        // Force disconnect and reconnect
        client.disconnect();
        BROKER_CONNECTED = false;
        mqtt_disconnect_noted = false;
        MQTT_FAIL_COUNT = 0;          // Reset for next connection attempt
        last_mqtt_send = currentTime; // Reset heartbeat timer
      }
    }
  }

// Handle sending refreshed port states via MQTT
void handleRefreshedPortStates() {
  static int current_refresh_port = 0;
  static unsigned long last_refresh_send = 0;
  
  // Check if we have any ports that need to send refresh updates
  if (current_refresh_port == 0) {
    // Find the first port that needs to send a refresh update
    for (int i = 1; i <= MAX_PORTS; i++) {
      if (isValidPort(i)) {
        PortState *portState = getPortState(i);
        if (portState && portState->send_refresh_port_update) {
          current_refresh_port = i;
          break;
        }
      }
    }
  }
  
  // If no port needs updates, exit
  if (current_refresh_port == 0) {
    return;
  }
  
  // Stagger MQTT messages to avoid flooding
  if (millis() - last_refresh_send < 500) { // 500ms delay between sends
    return;
  }
  
  PortState *portState = getPortState(current_refresh_port);
  if (!portState) {
    // Invalid port, move to next
    current_refresh_port++;
    if (current_refresh_port > MAX_PORTS) {
      current_refresh_port = 0; // Reset to find any other ports
    }
    return;
  }
  
  // Check if we're still waiting for data
  const unsigned long REFRESH_TIMEOUT = 20000; // 20 seconds timeout
  if (portState->await_refresh_port_update || 
      (portState->await_refresh_port_vin && portState->docked)) {
    // Check for timeout
    if (millis() - portState->refresh_request_time > REFRESH_TIMEOUT) {
      Serial.printlnf("Refresh timeout for port %d", current_refresh_port);
      // Timeout - clear flags and send what we have
      portState->await_refresh_port_update = false;
      portState->await_refresh_port_vin = false;
    } else {
      // Still waiting, will check again later
      return;
    }
  }
  
  // We have data or timed out - send the port state
  char portData[6] = "00000"; // Default all zeros
  
  // Format port data: Tag_Read_Success,Latch_Status,Secured,Charge_Status,FATAL_NFC_ERROR
  portData[0] = portState->valid_vehicle_tag ? '1' : '0';
  // Assuming latch_status is determined by docked and vehicle_secured
  portData[1] = (portState->docked && portState->vehicle_secured) ? '1' : '0';
  portData[2] = portState->vehicle_secured ? '1' : '0';
  portData[3] = portState->charging ? '1' : '0';
  portData[4] = portState->fatal_NFC_error ? '1' : '0';
  
  // Prepare MQTT message
  char message[64];
  snprintf(message, sizeof(message), "R,%d,%s,%s", 
           current_refresh_port, 
           portData, 
           portState->docked ? portState->VIN : "");
  
  // Send the message
  publishCloud(message);
  last_refresh_send = millis();
  
  // Clear the send flag
  portState->send_refresh_port_update = false;
  
  // Move to next port
  current_refresh_port++;
  if (current_refresh_port > MAX_PORTS) {
    current_refresh_port = 0; // Reset to find any other ports
  }
}
}

bool isMQTTHealthy() { return mqttHealthy && BROKER_CONNECTED; }

void forceMQTTReconnect() {
  Serial.println("Forcing MQTT reconnection...");
  client.disconnect();
  mqttReconnectTime = 0; // Force immediate reconnect attempt
  BROKER_CONNECTED = false;
  MQTT_FAIL_COUNT = 0;
  mqtt_disconnect_noted = false;
}

void checkPortStatusRequest() {
  if (!portStatusRequestPending || !portStatusWaitingForPoll) {
    return;
  }

  unsigned long currentTime = millis();
  // Wait for one full PORT_CHECK_INTERVAL plus buffer to ensure all ports
  // polled
  const unsigned long WAIT_TIME = (MAX_PORTS * POLL_STAGGER_DELAY) +
                                  5000; // PORT_CHECK_INTERVAL + 5s buffer

  if (currentTime - portStatusRequestTime >= WAIT_TIME) {
    Serial.printlnf(
        "Port status wait period complete (%lu ms, waited %lu ms) "
        "- sending response",
        WAIT_TIME, currentTime - portStatusRequestTime);
    portStatusWaitingForPoll = false;
    sendPortStatus();
    clearPortStatusRequest();
  } else {
    // Only log waiting message every 5 seconds to avoid spam
    static unsigned long lastWaitLog = 0;
    if (currentTime - lastWaitLog >= 5000) {
      unsigned long remainingWait =
          WAIT_TIME - (currentTime - portStatusRequestTime);
      Serial.printlnf("Waiting %lu more ms for fresh port data "
                      "(WAIT_TIME=%lu ms)...",
                      remainingWait, WAIT_TIME);
      lastWaitLog = currentTime;
    }
  }
}
