#include "cloud.h"
#include "Arduino.h"
#include "Particle.h"
#include "config.h"
#include "credentials.h"
#include "fixes/json_compat.h"
#include "lights.h"
#include "logging.h"
#include "port_event_handler.h"
#include "port_state.h"
#include "utils.h"
#include <ArduinoJson.h>

// Global MQTT variables
char MANUAL_MODE[14];
// Port status request variables
char portStatusRequest[64];
bool portStatusRequestPending = false;
unsigned long portStatusRequestTime = 0;
bool portStatusWaitingForPoll = false;

// Global message history for deduplication (MAX_PORTS + 1 for non-port messages
// at index 0)
struct MessageHistory messageHistory[MAX_PORTS + 1] = {0};

int juiceMessageCallback(String payload) {
  Serial.printlnf("Juice message received: %s", payload.c_str());

  // Convert String to char array for parsing
  int length = payload.length();
  char p[length + 1];
  payload.toCharArray(p, length + 1);

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
  processCloudCommand(cmd, variant, port, btn, tokens);
  return 1;
}

void processCloudCommand(char cmd, char variant, int port, char btn,
                         char *tokens[]) {
  struct PortState *portState = getPortState(port);

  switch (cmd) {
  case 'C': // Charge command
    if (isValidPort(port) && portState) {
      // Security validation: Only allow charging if properly authorized
      if (portEventHandler && portEventHandler->isChargingAuthorized(port)) {
        portState->check_charge_status = true;
        portState->charge_varient = '2'; // variant;
        portState->send_charge_flag = true;
        portState->awaiting_cloud_vin_resp = false;
        Serial.printlnf("Charge command authorized for port %d, variant %c\n",
                        port, variant);
      } else {
        Serial.printlnf("SECURITY VIOLATION: Charge command denied for port %d "
                        "- not authorized\n",
                        port);
      }
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

  case 'H': // Heartbeat command
    switch (variant) {
    case '0':
      if (tokens[3] && tokens[3][0] == '0') {
        // IoT heartbeat
        publishJuiseMessage("H,0,1");
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
        Serial.println("Invalid port number for emergency exit (VIN Invalid)");
      }
      break;
    }
    case '1': {
      Serial.println("VIN Validated, charging allowed");
      // Security validation: Only allow charging if properly authorized
      if (portEventHandler && portEventHandler->isChargingAuthorized(port)) {
        portState->check_charge_status = true;
        portState->charge_varient = '2'; // variant;
        portState->send_charge_flag = true;
        portState->awaiting_cloud_vin_resp = false;
        Serial.printlnf(
            "Cloud VIN validation successful - charging authorized for port %d",
            port);
      } else {
        Serial.printlnf("SECURITY VIOLATION: Cloud VIN validation received but "
                        "port %d not properly authorized",
                        port);
        portState->awaiting_cloud_vin_resp = false;
      }
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
  case 'S':
    if (isValidPort(port) && portState) {
      portState->send_manual_tag_read_flag = true;
      Serial.printlnf("Status Tag request for port %d\n", port);
    } else {
      Serial.printlnf("Invalid port for status request: %d\n", port);
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
    unsigned long requiredWait =
        PORT_CHECK_INTERVAL + (PORT_CHECK_INTERVAL / 2);
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
    publishJuiseMessage(retI); // Assuming publishCloud is defined elsewhere
  }
}

bool isPortStatusRequestPending() { return portStatusRequestPending; }

void clearPortStatusRequest() {
  portStatusRequestPending = false;
  portStatusWaitingForPoll = false;
  portStatusRequestTime = 0;
  memset(portStatusRequest, 0, sizeof(portStatusRequest));
}

void checkPortStatusRequest() {
  if (!portStatusRequestPending || !portStatusWaitingForPoll) {
    return;
  }

  unsigned long currentTime = millis();
  // Wait for one full PORT_CHECK_INTERVAL plus buffer to ensure all ports
  // polled
  const unsigned long WAIT_TIME =
      PORT_CHECK_INTERVAL + (PORT_CHECK_INTERVAL / 2);

  if (currentTime - portStatusRequestTime >= WAIT_TIME) {
    Serial.printlnf("Port status wait period complete (%lu ms, waited %lu ms) "
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
int publishJuiseMessage(const char *message) {
  // Deduplication logic to prevent sending the same message twice within 2
  // seconds

  // Try to extract port number from message
  // Message formats: "U,0,10,1" or "C,2,10,VIN" or "H,0,1" etc.
  int port = 0;
  char tempMessage[128];
  strncpy(tempMessage, message, sizeof(tempMessage) - 1);
  tempMessage[sizeof(tempMessage) - 1] = '\0';

  // Parse the message to get port number (3rd field in most messages)
  char *token = strtok(tempMessage, ",");
  if (token != NULL) {
    token = strtok(NULL, ","); // Skip to 2nd field
    if (token != NULL) {
      token = strtok(NULL, ","); // Get 3rd field (port number)
      if (token != NULL) {
        port = atoi(token);
        // Validate port range
        if (port < 0 || port > MAX_PORTS) {
          port = 0; // Use index 0 for invalid/non-port messages
        }
      }
    }
  }

  unsigned long currentTime = millis();

  // Check if this exact message was sent recently
  if (strcmp(messageHistory[port].lastMessage, message) == 0) {
    unsigned long timeSinceLastSent =
        currentTime - messageHistory[port].lastSentTime;
    if (timeSinceLastSent <
        MESSAGE_DEDUP_WINDOW_MS) { // Configurable deduplication window
      Serial.printlnf("DUPLICATE SUPPRESSED (sent %lu ms ago): %s",
                      timeSinceLastSent, message);
      return 1; // Return success but don't actually send
    }
  }

  // Store this message in history
  strncpy(messageHistory[port].lastMessage, message,
          sizeof(messageHistory[port].lastMessage) - 1);
  messageHistory[port]
      .lastMessage[sizeof(messageHistory[port].lastMessage) - 1] = '\0';
  messageHistory[port].lastSentTime = currentTime;

  String payload = String::format("{\"pub_id\":\"%s\",\"message\":\"%s\"}",
                                  MANUAL_MODE, message);
  Serial.printlnf("Publishing Juise message: %s", payload.c_str());
  return Particle.publish(JUISE_OUTGOING, payload.c_str(), PRIVATE);
}

void clearMessageHistory(int port) {
  if (port == 0) {
    // Clear all message history
    for (int i = 0; i <= MAX_PORTS; i++) {
      messageHistory[i].lastMessage[0] = '\0';
      messageHistory[i].lastSentTime = 0;
    }
    Serial.println("Cleared all message history for deduplication");
  } else if (port > 0 && port <= MAX_PORTS) {
    // Clear specific port's message history
    messageHistory[port].lastMessage[0] = '\0';
    messageHistory[port].lastSentTime = 0;
    Serial.printlnf("Cleared message history for port %d", port);
  }
}
