#include "port_state.h"
#include "Particle.h"
#include "can.h"
#include "config.h"
#include "mqtt.h"
#include "utils.h"

// Global port state variables
int CURRENT_PORT = 1;
struct PortState ports[MAX_PORTS];

// IoT State Variables
bool send_signal_flag = false;
bool send_heartbeat_flag = false;
bool send_iot_build_version_flag = false;

void initializePorts()
{
  // Initialize all ports to default state
  for (int i = 1; i < MAX_PORTS; i++)
  {
    resetPortState(i);
  }

  // Set current port to first port
  CURRENT_PORT = 1;

  // Initialize IoT state flags
  send_signal_flag = false;
  send_heartbeat_flag = false;
  send_iot_build_version_flag = false;
  memset(portStatusRequest, 0, sizeof(portStatusRequest));

  Serial.printlnf("Initialized %d ports\n", MAX_PORTS);
}
void markPortsUnpolled()
{
  Serial.println("TIME TO GET PORT DATA");
  for (int port = 1; port <= MAX_PORTS; port++)
  {
    PortState *state = &ports[port];
    if (state)
    {
      state->DID_PORT_CHECK = false;
    }
  }
}

void resetPortState(int portNumber)
{
  if (!isValidPort(portNumber))
  {
    Serial.printlnf("resetPortState Invalid port number: %d\n", portNumber);
    return;
  }

  int index = portNumber - 1; // Convert to 0-based index
  struct PortState *port = &ports[index];

  // Initialize message queue
  port->queueHead = 0;
  port->queueTail = 0;
  port->queueCount = 0;
  memset(port->messageQueue, 0, sizeof(port->messageQueue));

  // Reset all boolean flags
  port->DID_PORT_CHECK = false;
  port->docked = false;
  port->charging = false;
  port->valid_vehicle_tag = false;
  port->vehicle_secured = false;
  port->vin_request_flag = false;
  port->send_button_state_flag = false;
  port->emergency_exit_flag = false;
  port->send_port_build_version_flag = false;
  port->send_temp_req_flag = false;
  port->send_charging_params_flag = false;
  port->send_charge_flag = false;
  port->send_unlock_flag = false;
  port->send_vin_to_cloud_flag = false;
  port->awaiting_cloud_vin_resp = false;
  port->send_port_heartbeat = false;
  port->check_heartbeat_status = false;
  port->check_unlock_status = false;
  port->check_charge_status = false;
  port->unlock_successful = false;
  port->charge_successful = false;
  port->heartbeat_success = false;
  port->unlock_retry_count = 0;
  port->fatal_NFC_error = false;

  // Reset numeric values
  port->command_timeout = 0;
  port->send_vin_request_timer = 0;
  port->last_poll_time = 0;
  port->cloud_vin_resp_timer = 0;

  // Reset character values
  port->button_state = 0;
  port->charge_varient = 0;

  // Reset string arrays
  memset(port->VIN, 0, sizeof(port->VIN));
  memset(port->volts, 0, sizeof(port->volts));
  memset(port->amps, 0, sizeof(port->amps));
  memset(port->temp, 0, sizeof(port->temp));
  memset(port->fan_speed, 0, sizeof(port->fan_speed));
  memset(port->port_firmware_version, 0, sizeof(port->port_firmware_version));
}

void updatePortState(int portNumber, const struct PortState *newState)
{
  if (!isValidPort(portNumber) || newState == NULL)
  {
    Serial.printlnf("Invalid port number or null state: %d\n", portNumber);
    return;
  }

  int index = portNumber - 1; // Convert to 0-based index
  memcpy(&ports[index], newState, sizeof(struct PortState));

  // Update last poll time
  ports[index].last_poll_time = millis();

  Serial.printlnf("Updated port %d state\n", portNumber);
}

struct PortState *getPortState(int portNumber)
{
  if (!isValidPort(portNumber))
  {
    Serial.printlnf("getPortState Invalid port number: %d\n", portNumber);
    return NULL;
  }

  int index = portNumber - 1; // Convert to 0-based index
  return &ports[index];
}

bool isValidPort(int portNumber)
{
  return (portNumber >= 1 && portNumber <= MAX_PORTS);
}

void setCurrentPort(int portNumber)
{
  if (isValidPort(portNumber))
  {
    CURRENT_PORT = portNumber;
    Serial.printlnf("Current port set to: %d\n", CURRENT_PORT);
  }
  else
  {
    Serial.printlnf("Invalid port number for current port: %d\n", portNumber);
  }
}

int getCurrentPort() { return CURRENT_PORT; }

bool isPortDocked(int portNumber)
{
  struct PortState *port = getPortState(portNumber);
  return port ? port->docked : false;
}

bool isPortCharging(int portNumber)
{
  struct PortState *port = getPortState(portNumber);
  return port ? port->charging : false;
}

bool isPortSecured(int portNumber)
{
  struct PortState *port = getPortState(portNumber);
  return port ? port->vehicle_secured : false;
}

void setPortVIN(int portNumber, const char *vin)
{
  struct PortState *port = getPortState(portNumber);
  if (port && vin)
  {
    safeStrCopy(port->VIN, vin, sizeof(port->VIN));
    Serial.printlnf("Port %d VIN set to: %s\n", portNumber, port->VIN);
  }
}

const char *getPortVIN(int portNumber)
{
  struct PortState *port = getPortState(portNumber);
  return port ? port->VIN : "";
}

void setPortTemperature(int portNumber, const char *temperature)
{
  struct PortState *port = getPortState(portNumber);
  if (port && temperature)
  {
    safeStrCopy(port->temp, temperature, sizeof(port->temp));
    Serial.printlnf("Port %d temperature set to: %s\n", portNumber, port->temp);
  }
}

void setPortVoltage(int portNumber, const char *voltage)
{
  struct PortState *port = getPortState(portNumber);
  if (port && voltage)
  {
    safeStrCopy(port->volts, voltage, sizeof(port->volts));
    Serial.printlnf("Port %d voltage set to: %s\n", portNumber, port->volts);
  }
}

void setPortCurrent(int portNumber, const char *current)
{
  struct PortState *port = getPortState(portNumber);
  if (port && current)
  {
    safeStrCopy(port->amps, current, sizeof(port->amps));
    Serial.printlnf("Port %d current set to: %s\n", portNumber, port->amps);
  }
}

void setPortFirmwareVersion(int portNumber, const char *version)
{
  struct PortState *port = getPortState(portNumber);
  if (port && version)
  {
    safeStrCopy(port->port_firmware_version, version,
                sizeof(port->port_firmware_version));
    Serial.printlnf("Port %d firmware version set to: %s\n", portNumber,
                    port->port_firmware_version);
  }
}

String getPortStatusSummary(int portNumber)
{
  struct PortState *port = getPortState(portNumber);
  if (!port)
  {
    return "Invalid port";
  }

  String status = "Port " + String(portNumber) + ": ";
  status += port->docked ? "Docked" : "Empty";

  if (port->docked)
  {
    status += port->charging ? ", Charging" : ", Not Charging";
    status += port->vehicle_secured ? ", Secured" : ", Unsecured";

    if (strlen(port->VIN) > 0)
    {
      status += ", VIN: " + String(port->VIN);
    }
  }

  return status;
}

unsigned long getPortLastPollTime(int portNumber)
{
  struct PortState *port = getPortState(portNumber);
  return port ? port->last_poll_time : 0;
}

void markPortPolled(int portNumber)
{
  struct PortState *port = getPortState(portNumber);
  if (port)
  {
    port->last_poll_time = millis();
    port->DID_PORT_CHECK = true;
  }
  if (isPortStatusRequestPending())
  {
    sendPortStatus();
    clearPortStatusRequest();
  }
}

bool hasPortBeenPolled(int portNumber)
{
  struct PortState *port = getPortState(portNumber);
  return port ? port->DID_PORT_CHECK : false;
}

void clearPortFlags(int portNumber)
{
  struct PortState *port = getPortState(portNumber);
  if (!port)
    return;

  // Clear all command flags
  port->vin_request_flag = false;
  port->send_button_state_flag = false;
  port->emergency_exit_flag = false;
  port->send_port_build_version_flag = false;
  port->send_temp_req_flag = false;
  port->send_charging_params_flag = false;
  port->send_charge_flag = false;
  port->send_unlock_flag = false;
  port->send_vin_to_cloud_flag = false;
  port->send_port_heartbeat = false;
  port->check_heartbeat_status = false;
  port->check_unlock_status = false;
  port->check_charge_status = false;
  port->unlock_retry_count = 0;

  Serial.printlnf("Cleared flags for port %d\n", portNumber);
}

char *getPortStatusRange(int startPort, int endPort)
{
  static char buffer[256]; // Static buffer to return
  memset(buffer, 0, sizeof(buffer));

  if (startPort < 1 || endPort > MAX_PORTS || startPort > endPort)
  {
    Serial.printlnf("Invalid port range: %d-%d", startPort, endPort);
    snprintf(buffer, sizeof(buffer), "P,0,ERROR");
    return buffer;
  }

  // Start building the response: P,0,status1,status2,status3,...
  strcpy(buffer, "P,0");

  for (int port = startPort; port <= endPort; port++)
  {
    struct PortState *portState = getPortState(port);
    char portStatus[8];

    if (portState)
    {
      // Format: just the status value (1=docked, 0=empty)
      snprintf(portStatus, sizeof(portStatus), ",%d",
               portState->docked ? 1 : 0);
    }
    else
    {
      // Invalid port - use 0
      snprintf(portStatus, sizeof(portStatus), ",0");
    }

    // Check if adding this would exceed buffer size
    if (strlen(buffer) + strlen(portStatus) < sizeof(buffer) - 1)
    {
      strcat(buffer, portStatus);
    }
    else
    {
      Serial.println("Port status buffer overflow prevented");
      break;
    }
  }

  Serial.printlnf("Port status range %d-%d: %s", startPort, endPort, buffer);
  return buffer;
}

// Message delay management functions
bool canSendMessageToPort(int portNumber)
{
  PortState *state = getPortState(portNumber);
  if (!state)
  {
    return false;
  }

  unsigned long currentTime = millis();
  unsigned long timeSinceLastMessage = currentTime - state->last_message_time;

  // If no message has been sent yet (last_message_time = 0) or enough time has
  // passed
  bool canSend = (state->last_message_time == 0 ||
                  timeSinceLastMessage >= SUBSEQUENT_MSG_DELAY);

  if (!canSend)
  {
    Serial.printlnf(
        "Port %d delay check: last_msg=%lu ms ago, need=%lu ms, waiting=%lu ms",
        portNumber, timeSinceLastMessage, SUBSEQUENT_MSG_DELAY,
        SUBSEQUENT_MSG_DELAY - timeSinceLastMessage);
  }

  return canSend;
}

void markMessageSentToPort(int portNumber)
{
  PortState *state = getPortState(portNumber);
  if (state)
  {
    unsigned long oldTime = state->last_message_time;
    state->last_message_time = millis();
    Serial.printlnf(
        "Port %d message timestamp updated: %lu -> %lu (gap: %lu ms)",
        portNumber, oldTime, state->last_message_time,
        oldTime > 0 ? state->last_message_time - oldTime : 0);
  }
}

unsigned long getTimeSinceLastMessage(int portNumber)
{
  PortState *state = getPortState(portNumber);
  if (!state || state->last_message_time == 0)
  {
    return SUBSEQUENT_MSG_DELAY; // Return a value >= delay if no previous
                                 // message
  }

  unsigned long currentTime = millis();
  return currentTime - state->last_message_time;
}

// Message queue management functions
bool queueMessageForPort(int portNumber, char command, const char *variant,
                         int timeout)
{
  PortState *state = getPortState(portNumber);
  if (!state)
  {
    Serial.printlnf("Cannot queue message for invalid port %d", portNumber);
    return false;
  }

  // Check if queue is full
  if (state->queueCount >= MAX_QUEUED_MESSAGES_PER_PORT)
  {
    Serial.printlnf("Message queue full for port %d, dropping message",
                    portNumber);
    return false;
  }

  // Add message to queue
  QueuedMessage *msg = &state->messageQueue[state->queueTail];
  msg->command = command;
  msg->timeout = timeout;
  msg->queueTime = millis();
  msg->hasVariant = (variant != nullptr);

  if (variant != nullptr)
  {
    safeStrCopy(msg->variant, variant, sizeof(msg->variant));
  }
  else
  {
    msg->variant[0] = '\0';
  }

  // Update queue indices
  state->queueTail = (state->queueTail + 1) % MAX_QUEUED_MESSAGES_PER_PORT;
  state->queueCount++;

  Serial.printlnf(
      "QUEUED: Port %d message '%c' (queue size: %d, delay needed: %lu ms)",
      portNumber, command, state->queueCount,
      canSendMessageToPort(portNumber)
          ? 0
          : SUBSEQUENT_MSG_DELAY - getTimeSinceLastMessage(portNumber));
  return true;
}

bool hasQueuedMessages(int portNumber)
{
  PortState *state = getPortState(portNumber);
  return state && (state->queueCount > 0);
}

bool sendNextQueuedMessage(int portNumber)
{
  PortState *state = getPortState(portNumber);
  if (!state || state->queueCount == 0)
  {
    return false;
  }

  // Check if we can send to this port yet
  if (!canSendMessageToPort(portNumber))
  {
    return false; // Not ready yet, try again later
  }

  // Get the next message
  QueuedMessage *msg = &state->messageQueue[state->queueHead];

  Serial.printlnf("Sending queued message '%c' to port %d", msg->command,
                  portNumber);

  // Build and send CAN message directly to avoid circular dependency
  struct can_frame reqMsg;
  memset(reqMsg.data, 0, sizeof(reqMsg.data));

  reqMsg.can_id = portNumber;
  reqMsg.data[0] = (unsigned char)msg->command;
  reqMsg.data[1] = ',';

  // Convert port to string
  char portStr[3];
  snprintf(portStr, sizeof(portStr), "%d", portNumber);
  size_t portStrLen = strlen(portStr);

  // Copy port number string into message
  for (size_t i = 0; i < portStrLen && i < 2; i++)
  {
    reqMsg.data[2 + i] = portStr[i];
  }

  if (msg->hasVariant)
  {
    // Add comma separator after port number
    reqMsg.data[2 + portStrLen] = ',';

    // Add variant
    size_t maxVariantLen = sizeof(reqMsg.data) - (3 + portStrLen) - 1;
    safeStrCopy((char *)&reqMsg.data[3 + portStrLen], msg->variant,
                maxVariantLen);
    reqMsg.data[7] = '\0';
    reqMsg.can_dlc = 8;
  }
  else
  {
    // Null-pad the remaining bytes
    for (size_t i = 2 + portStrLen; i < 8; i++)
    {
      reqMsg.data[i] = '\0';
    }
    reqMsg.can_dlc = 8;
  }

  // Send the CAN message
  int result = sendCanMessage(reqMsg);

  if (result == ERROR_OK)
  {
    markMessageSentToPort(portNumber);
    Serial.printlnf("DEQUEUED: Port %d message '%c' sent successfully",
                    portNumber, msg->command);
  }
  else
  {
    Serial.printlnf("DEQUEUE_FAILED: Port %d message '%c' failed, error: %d",
                    portNumber, msg->command, result);
  }

  // Remove message from queue regardless of send result
  state->queueHead = (state->queueHead + 1) % MAX_QUEUED_MESSAGES_PER_PORT;
  state->queueCount--;

  Serial.printlnf("Message processed for port %d, queue size now: %d",
                  portNumber, state->queueCount);
  return (result == ERROR_OK);
}

void processPortMessageQueues()
{
  // Process queues for all ports
  for (int port = 1; port <= MAX_PORTS; port++)
  {
    if (hasQueuedMessages(port))
    {
      sendNextQueuedMessage(port);
    }
  }
}

int getQueuedMessageCount(int portNumber)
{
  PortState *state = getPortState(portNumber);
  return state ? state->queueCount : 0;
}

void clearPortMessageQueue(int portNumber)
{
  PortState *state = getPortState(portNumber);
  if (state)
  {
    state->queueHead = 0;
    state->queueTail = 0;
    state->queueCount = 0;
    memset(state->messageQueue, 0, sizeof(state->messageQueue));
    Serial.printlnf("Cleared message queue for port %d", portNumber);
  }
}
