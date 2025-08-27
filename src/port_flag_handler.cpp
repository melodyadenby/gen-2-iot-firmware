#include "port_flag_handler.h"
#include "can.h"
#include "mqtt.h"
#include "port_event_handler.h"
#include "port_state.h"
#include "utils.h"
#include <string.h>

// Global instance declared in main.h, defined in main.ino

PortFlagHandler::PortFlagHandler(PortStateManager *manager)
    : portStateManager(manager), currentPort(1) {}

void PortFlagHandler::processAllPortFlags() {
  // Process flags for all ports in round-robin fashion
  for (int i = 0; i < MAX_PORTS; i++) {
    int port = getNextPort();
    if (hasPortPendingFlags(port)) {
      processPortFlags(port);
    }
  }
}

void PortFlagHandler::processPortFlags(int port) {
  if (!isValidPort(port)) {
    return;
  }

  PortState *state = getPortState(port);
  if (!state) {
    return;
  }

  // Check for partial VIN timeout
  checkVINTimeout(port, state);

  // logFlagActivity(port, "PROCESSING", "Starting flag processing");

  // Process command flags in priority order
  handleEmergencyExit(port);
  handleVINRequest(port);
  handleUnlockCommand(port);
  handleChargeCommand(port);
  handleHeartbeat(port);
  handleTemperatureRequest(port);
  handleManualTagRead(port);
  handleChargingParameters(port);
  handlePortVersionRequest(port);
  handleVINToCloud(port);
  handleButtonState(port);

  // Check status flags
  checkUnlockStatus(port);
  checkChargeStatus(port);
  checkHeartbeatStatus(port);

  // Handle timeouts
  checkCommandTimeouts(port);
}

void PortFlagHandler::handleVINRequest(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->vin_request_flag) {
    return;
  }

  logFlagActivity(port, "VIN_REQUEST", "Sending VIN request");

  if (sendPortCommand(port, 'K', nullptr, 10 * SEC_TO_MS_MULTIPLIER) ==
      ERROR_OK) {
    state->vin_request_flag = false;
    state->send_vin_request_timer = millis();
  } else {
    handleCommandError(port, "VIN_REQUEST", -1);
  }
}

void PortFlagHandler::handleUnlockCommand(int port) {
  PortState *state = getPortState(port);
  if (!state) {
    return;
  }

  if (state->emergency_exit_flag) {
    // Check if enough time has passed since last emergency unlock
    unsigned long currentTime = millis();
    if (currentTime - state->last_emergency_unlock_time <
        EMERGENCY_UNLOCK_DELAY) {
      return; // Still in delay period, skip this attempt
    }

    logFlagActivity(port, "EMERGENCY_EXIT", "Processing emergency unlock");

    if (sendPortCommand(port, 'U', nullptr, 3 * SEC_TO_MS_MULTIPLIER) ==
        ERROR_OK) {
      state->send_unlock_flag = false;
      state->check_unlock_status = true;
      state->last_emergency_unlock_time = currentTime;
    } else {
      handleCommandError(port, "EMERGENCY_UNLOCK", -1);
    }
  } else if (state->send_unlock_flag) {
    logFlagActivity(port, "UNLOCK", "Sending unlock command");

    if (sendPortCommand(port, 'U', nullptr, 3 * SEC_TO_MS_MULTIPLIER) ==
        ERROR_OK) {
      state->send_unlock_flag = false;
      state->check_unlock_status = true;
    } else {
      handleCommandError(port, "UNLOCK", -1);
    }
  }
}

void PortFlagHandler::handleChargeCommand(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->send_charge_flag) {
    return;
  }

  // Security validation: Only allow charging if properly authorized
  if (portEventHandler && !portEventHandler->isChargingAuthorized(port)) {
    Serial.printlnf(
        "SECURITY VIOLATION: Cannot charge port %d - not authorized!", port);
    Serial.printlnf(
        "Port %d - Charge command blocked to prevent unauthorized charging. "
        "Vehicle will be ejected after grace period if VIN not validated.",
        port);
    state->send_charge_flag = false;
    state->check_charge_status = false;
    return;
  }

  logFlagActivity(port, "CHARGE", "Sending charge command");
  Serial.printlnf("Port %d - Charging authorized for secured vehicle with VIN",
                  port);

  // Convert single char to string
  char variantStr[2] = {state->charge_varient, '\0'};

  if (sendPortCommand(port, 'C', variantStr, 2 * PORT_CHECK_INTERVAL) ==
      ERROR_OK) {
    state->send_charge_flag = false;
    state->check_charge_status = true;
  } else {
    handleCommandError(port, "CHARGE", -1);
  }
}

void PortFlagHandler::handleHeartbeat(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->send_port_heartbeat) {
    return;
  }

  logFlagActivity(port, "HEARTBEAT", "Sending heartbeat");

  if (sendPortCommand(port, 'H', nullptr, 5 * SEC_TO_MS_MULTIPLIER) ==
      ERROR_OK) {
    state->send_port_heartbeat = false;
    state->check_heartbeat_status = true;
  } else {
    handleCommandError(port, "HEARTBEAT", -1);
  }
}

void PortFlagHandler::handleTemperatureRequest(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->send_temp_req_flag) {
    return;
  }

  logFlagActivity(port, "TEMPERATURE", "Requesting temperature data");

  if (sendPortCommand(port, 'T', "0", 10 * SEC_TO_MS_MULTIPLIER) == ERROR_OK) {
    state->send_temp_req_flag = false;
  } else {
    handleCommandError(port, "TEMPERATURE", -1);
  }
}
void PortFlagHandler::handleManualTagRead(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->send_manual_tag_read_flag) {
    return;
  }

  logFlagActivity(port, "TAG STATUS", "Requesting Tag data");
  Serial.printlnf("Status Tag request for port %d\n", port);
  if (sendPortCommand(port, 'S', "0", 10 * SEC_TO_MS_MULTIPLIER) == ERROR_OK) {
    Serial.println("Tag data requested successfully");
    state->send_manual_tag_read_flag = false;
  } else {
    handleCommandError(port, "TAG STATUS", -1);
  }
}

void PortFlagHandler::handleChargingParameters(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->send_charging_params_flag) {
    return;
  }

  logFlagActivity(port, "CHARGE_PARAMS", "Sending charging parameters");

  if (sendChargingParams(port, state->volts, state->amps, 1) == ERROR_OK) {
    state->send_charging_params_flag = false;
  } else {
    handleCommandError(port, "CHARGE_PARAMS", -1);
    Serial.println("Failed to send port params!");
  }
}

void PortFlagHandler::handlePortVersionRequest(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->send_port_build_version_flag) {
    return;
  }

  logFlagActivity(port, "VERSION", "Requesting port version");

  if (sendPortCommand(port, 'V', nullptr, 10 * SEC_TO_MS_MULTIPLIER) ==
      ERROR_OK) {
    state->send_port_build_version_flag = false;
  } else {
    handleCommandError(port, "VERSION", -1);
  }
}

void PortFlagHandler::handleEmergencyExit(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->emergency_exit_flag) {
    return;
  }

  // Check if enough time has passed since last emergency unlock
  unsigned long currentTime = millis();
  if (state->last_emergency_unlock_time != 0 &&
      currentTime - state->last_emergency_unlock_time <
          EMERGENCY_UNLOCK_DELAY) {
    return; // Still in delay period, skip this attempt
  }

  logFlagActivity(port, "EMERGENCY_EXIT", "Processing emergency exit");
  state->send_unlock_flag = true;
}

void PortFlagHandler::handleVINToCloud(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->send_vin_to_cloud_flag) {
    return;
  }

  logFlagActivity(port, "VIN_TO_CLOUD", "Sending VIN to cloud");

  if (strlen(state->VIN) > 0) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "C,2,%d,%s", port, state->VIN);
    publishToCloud(buffer);

    // Clear all VIN cloud flags - no retry logic needed
    state->send_vin_to_cloud_flag = false;
    state->awaiting_cloud_vin_resp = false;
    state->cloud_vin_resp_timer = 0;

    Serial.printlnf("Port %d - VIN sent to cloud (one-time only): %s", port,
                    state->VIN);
  }
}

void PortFlagHandler::handleButtonState(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->send_button_state_flag) {
    return;
  }

  logFlagActivity(port, "BUTTON_STATE", "Sending button state");
  // TEMP REMOVE
  //  char buffer[32];
  //  snprintf(buffer, sizeof(buffer), "BUTTON,%d,%c", port,
  //  state->button_state); publishToCloud(buffer);

  state->send_button_state_flag = false;
}

void PortFlagHandler::checkCommandTimeouts(int port) {
  PortState *state = getPortState(port);
  if (!state) {
    return;
  }

  // Check if timeout has expired (compare with current time)
  if (state->command_timeout > 0) {
    unsigned long currentTime = millis();
    if (currentTime >= state->command_timeout) {
      // Timeout expired
      state->command_timeout = 0;
    }
  }
}

void PortFlagHandler::checkUnlockStatus(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->check_unlock_status) {
    return;
  }

  if (state->unlock_successful) {
    handleUnlockSuccess(port);
  } else if (state->command_timeout <= 0) {
    handleUnlockFailure(port);
  }
}

void PortFlagHandler::checkChargeStatus(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->check_charge_status) {
    return;
  }

  if (state->charge_successful) {
    handleChargeSuccess(port);
  } else if (state->command_timeout <= 0) {
    handleChargeFailure(port);
  }
}

void PortFlagHandler::checkHeartbeatStatus(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->check_heartbeat_status) {
    return;
  }

  if (state->heartbeat_success) {
    logFlagActivity(port, "HEARTBEAT", "Success");

    // Publish heartbeat success to cloud: H,0,port,1
    char buffer[16];
    formatCloudMessage("H", "0", port, "1", buffer, sizeof(buffer));
    publishToCloud(buffer);

    state->check_heartbeat_status = false;
    state->heartbeat_success = false;
  } else if (state->command_timeout <= 0) {
    logFlagActivity(port, "HEARTBEAT", "Timeout");

    // Publish heartbeat failure to cloud: H,0,port,0
    char buffer[16];
    formatCloudMessage("H", "0", port, "0", buffer, sizeof(buffer));
    publishToCloud(buffer);

    state->check_heartbeat_status = false;
  }
}

void PortFlagHandler::handleUnlockSuccess(int port) {
  logFlagActivity(port, "UNLOCK", "Success");

  char buffer[16];
  formatCloudMessage("U", "0", port, "1", buffer, sizeof(buffer));
  publishToCloud(buffer);

  // Reset retry count on success
  PortState *state = getPortState(port);
  if (state) {
    state->unlock_retry_count = 0;
  }

  resetPortAfterOperation(port);
}

void PortFlagHandler::handleUnlockFailure(int port) {
  PortState *state = getPortState(port);
  if (!state) {
    return;
  }

  // Increment retry count
  state->unlock_retry_count++;

  // Check if we should retry
  if (state->unlock_retry_count < MAX_UNLOCK_RETRY) {
    char retryMessage[64];
    snprintf(retryMessage, sizeof(retryMessage),
             "Failed - timeout, retry %d/%d", state->unlock_retry_count,
             MAX_UNLOCK_RETRY);
    logFlagActivity(port, "UNLOCK", retryMessage);

    // Reset flags to retry the unlock command
    state->send_unlock_flag = true;
    state->check_unlock_status = false;
    state->command_timeout = 0; // Clear timeout to allow immediate retry
    return;
  }

  // Max retries reached, report final failure
  char failureMessage[64];
  snprintf(failureMessage, sizeof(failureMessage),
           "Failed - max retries (%d) reached", MAX_UNLOCK_RETRY);
  logFlagActivity(port, "UNLOCK", failureMessage);

  char buffer[16];
  formatCloudMessage("U", "0", port, "0", buffer, sizeof(buffer));
  publishToCloud(buffer);

  // Reset all unlock-related flags
  state->check_unlock_status = false;
  state->unlock_retry_count = 0;
  state->DID_PORT_CHECK = false;
}

void PortFlagHandler::handleChargeSuccess(int port) {
  logFlagActivity(port, "CHARGE", "Success");

  PortState *state = getPortState(port);
  if (state) {
    // // Publish charge success to cloud: C,variant,port,1
    // char buffer[16];
    // char variantStr[2] = {state->charge_varient, '\0'};
    // formatCloudMessage("C", variantStr, port, "1", buffer, sizeof(buffer));
    // // publishToCloud(buffer);

    state->check_charge_status = false;
    state->charge_successful = false;
    state->DID_PORT_CHECK = false;
    state->charge_varient = '\0';
  }
}

void PortFlagHandler::handleChargeFailure(int port) {
  logFlagActivity(port, "CHARGE", "Failed - timeout");

  PortState *state = getPortState(port);
  if (state) {
    // // Publish charge failure to cloud: C,variant,port,0
    // char buffer[16];
    // char variantStr[2] = {state->charge_varient, '\0'};
    // formatCloudMessage("C", variantStr, port, "0", buffer, sizeof(buffer));
    // // publishToCloud(buffer);

    state->check_charge_status = false;
    state->charge_successful = false;
    state->DID_PORT_CHECK = false;
    state->charge_varient = '\0';
  }
}

int PortFlagHandler::sendPortCommand(int port, char command,
                                     const char *variant, int timeout) {
  Serial.printlnf("GOING TO SEND MESSAGE FOR PORT %d", port);
  if (!isValidPort(port)) {
    return -1;
  }

  return portWrite(port, command, const_cast<char *>(variant), timeout);
}

int PortFlagHandler::sendChargingParams(int port, const char *volts,
                                        const char *amps, int timeout) {
  if (!isValidPort(port)) {
    return -1;
  }

  // Use existing portWriteParams function
  return portWriteParams(port, const_cast<char *>(volts),
                         const_cast<char *>(amps), timeout);
}

void PortFlagHandler::publishToCloud(const char *message) {
  if (isMQTTConnected()) {
    publishCloud(String(message));
  } else {
    // Fallback to Particle cloud
    Particle.publish("port_status", message, PRIVATE);
  }
}

void PortFlagHandler::resetPortAfterOperation(int port) {
  PortState *state = getPortState(port);
  if (state) {
    state->check_unlock_status = false;
    state->emergency_exit_flag = false;
    state->unlock_successful = false;
    state->send_vin_to_cloud_flag = false;
    state->send_vin_request_timer = 0;
    state->DID_PORT_CHECK = false;
    state->vin_request_flag = false;

    // Clear VIN cloud response state
    state->awaiting_cloud_vin_resp = false;
    state->cloud_vin_resp_timer = 0;

    // Clear VIN and charge variant only on unlock (vehicle leaving)
    memset(state->VIN, 0, sizeof(state->VIN));
    state->charge_varient = '\0';

    Serial.printlnf("Port %d - Port state reset after operation, VIN cleared",
                    port);
  }
}

void PortFlagHandler::logFlagActivity(int port, const char *flagName,
                                      const char *action) {
  Serial.printlnf("Port %d - %s: %s", port, flagName, action);
}

int PortFlagHandler::getNextPort() {
  currentPort++;
  if (currentPort > MAX_PORTS) {
    currentPort = 1;
  }
  return currentPort;
}

bool PortFlagHandler::hasPortPendingFlags(int port) {
  PortState *state = getPortState(port);
  if (!state) {
    return false;
  }

  return (state->vin_request_flag || state->send_unlock_flag ||
          state->send_charge_flag || state->send_port_heartbeat ||
          state->send_temp_req_flag || state->send_manual_tag_read_flag ||
          state->send_charging_params_flag ||
          state->send_port_build_version_flag || state->emergency_exit_flag ||
          state->send_vin_to_cloud_flag || state->send_button_state_flag ||
          state->check_unlock_status || state->check_charge_status ||
          state->check_heartbeat_status);
}

int PortFlagHandler::getPendingPortsCount() {
  int count = 0;
  for (int port = 1; port <= MAX_PORTS; port++) {
    if (hasPortPendingFlags(port)) {
      count++;
    }
  }
  return count;
}

bool PortFlagHandler::isValidPort(int port) {
  return (port >= 1 && port <= MAX_PORTS);
}

void PortFlagHandler::formatCloudMessage(const char *command,
                                         const char *variant, int port,
                                         const char *success, char *buffer,
                                         size_t bufferSize) {
  snprintf(buffer, bufferSize, "%s,%s,%d,%s", command, variant, port, success);
}

void PortFlagHandler::handleCommandError(int port, const char *command,
                                         int errorCode) {
  Serial.printlnf("CAN ERROR on port %d for command %s (error: %d)", port,
                  command, errorCode);

  // char buffer[64];
  // snprintf(buffer, sizeof(buffer), "CAN_ERROR,%d,%s,%d", port, command,
  //          errorCode);
  // publishToCloud(buffer);
}

bool PortFlagHandler::canRetryCommand(unsigned long lastAttemptTime,
                                      unsigned long retryInterval) {
  return (millis() - lastAttemptTime) >= retryInterval;
}

void PortFlagHandler::updateCommandTimeout(int port, int decrement) {
  // This function is no longer needed since we use absolute timestamps
  // Keeping for compatibility but making it a no-op
}

void PortFlagHandler::checkVINTimeout(int port, PortState *state) {
  const unsigned long VIN_TIMEOUT = 30000; // 30 seconds

  // Only check if we have a partial VIN
  if (strlen(state->VIN) > 0 && strlen(state->VIN) < VIN_LENGTH) {
    unsigned long timeSinceLastUpdate =
        millis() - state->send_vin_request_timer;
    if (timeSinceLastUpdate > VIN_TIMEOUT) {
      Serial.printlnf("Port %d - Partial VIN timeout after %lu ms, clearing "
                      "and restarting",
                      port, timeSinceLastUpdate);

      // Clear partial VIN and restart the process
      memset(state->VIN, 0, sizeof(state->VIN));
      state->vin_request_flag = true;
      state->send_vin_request_timer = millis();

      Serial.printlnf("Port %d - VIN timeout recovery: requesting new VIN",
                      port);
    }
  }
}

int PortFlagHandler::portWriteParams(int port, char volts[], char amps[],
                                     int timeout) {
  Serial.printf("Sending charge params - volts: %s, amps: %s\n", volts, amps);

  struct PortState *portState = getPortState(port);
  if (!portState) {
    return -1;
  }
  // Set absolute timeout timestamp (current time + timeout duration)
  portState->command_timeout = millis() + timeout;

  struct can_frame reqMsg;
  memset(&reqMsg, 0, sizeof(reqMsg));

  reqMsg.can_id = port;
  reqMsg.can_dlc = 0;

  reqMsg.data[reqMsg.can_dlc++] = 'P';
  reqMsg.data[reqMsg.can_dlc++] = ',';

  // Add volts
  for (int i = 0; i < strlen(volts) && reqMsg.can_dlc < 8; i++) {
    reqMsg.data[reqMsg.can_dlc++] = volts[i];
  }

  reqMsg.data[reqMsg.can_dlc++] = ',';

  // Add amps
  for (int i = 0; i < strlen(amps) && reqMsg.can_dlc < 8; i++) {
    reqMsg.data[reqMsg.can_dlc++] = amps[i];
  }

  int result = sendCanMessage(reqMsg);
  if (result != ERROR_OK) {
    Serial.printlnf("CAN Error in portWriteParams: %d", result);
  }
  return result;
}

int PortFlagHandler::portWrite(int port, char cmd, char *variant, int timeout) {
  struct PortState *portState = getPortState(port);

  Serial.printlnf("portWriteNew: port=%d, cmd=%c", port, cmd);

  struct can_frame reqMsg;
  if (port < 1 || port > MAX_PORTS) {
    Serial.printf("portWrite Invalid port number: %d\n", port);
    return -1; // Return negative value to indicate error
  }

  // Set absolute timeout timestamp (current time + timeout duration)
  portState->command_timeout = millis() + timeout;

  // Clear the CAN message buffer to avoid leftover data
  memset(reqMsg.data, 0, sizeof(reqMsg.data));

  reqMsg.can_id = port;

  // Build the CAN message
  reqMsg.data[0] = (unsigned char)cmd;
  reqMsg.data[1] = ',';

  // Convert port to string for both single and double digit ports
  char portStr[3];
  snprintf(portStr, sizeof(portStr), "%d", port);
  size_t portStrLen = strlen(portStr);

  // Copy port number string into message (with bounds checking)
  size_t maxPortDigits = 2; // Maximum digits in port number
  for (size_t i = 0; i < portStrLen && i < maxPortDigits; i++) {
    reqMsg.data[2 + i] = portStr[i];
  }

  if (variant != NULL) {
    // Add comma separator after port number
    reqMsg.data[2 + portStrLen] = ',';

    // Calculate max safe length for variant
    size_t maxVariantLen =
        sizeof(reqMsg.data) - (3 + portStrLen) - 1; // -1 for null terminator

    // Use our safe string copy function
    safeStrCopy((char *)&reqMsg.data[3 + portStrLen], variant, maxVariantLen);

    // Ensure null-termination of CAN message
    reqMsg.data[7] = '\0';

    // Set the CAN message length to 8
    reqMsg.can_dlc = 8;
  } else {
    // Null-pad the remaining bytes starting after port number
    for (size_t i = 2 + portStrLen; i < 8; i++) {
      reqMsg.data[i] = '\0';
    }
    Serial.print("Sending to port: ");
    Serial.println(port);
    Serial.printf("Data: %s", reqMsg.data);
    Serial.println();
    // Set the CAN message length to 8
    reqMsg.can_dlc = 8;
  }

  // Attempt to send the message
  int result = sendCanMessage(reqMsg);

  // Check for send errors
  if (result != ERROR_OK) {
    Serial.printlnf("CAN send error: %d when sending to port %d", result, port);
  }

  return result;
}

bool PortFlagHandler::sendGetPortData(int addr) {
  if (addr == 0) { // Address 0 is reserved for the IoT device itself
    return true;
  }

  if (!isValidPort(addr)) {
    Serial.printlnf("Invalid port number for data request: %d", addr);
    return false;
  }

  struct can_frame reqMsg;

  // Clear the entire CAN message buffer to avoid leftover data
  memset(reqMsg.data, 0, sizeof(reqMsg.data));

  reqMsg.can_id = addr;

  // Construct the message: "R,<port>"
  reqMsg.data[0] = 'R'; // Command prefix
  reqMsg.data[1] = ',';

  // Convert port to string for both single and double digit ports
  char portStr[3];
  snprintf(portStr, sizeof(portStr), "%d", addr);
  size_t portStrLen = strlen(portStr);

  // Copy port number string into message
  for (size_t i = 0; i < portStrLen; i++) {
    reqMsg.data[2 + i] = portStr[i];
  }

  // Null-terminate the message and pad remaining bytes
  reqMsg.data[2 + portStrLen] = '\0';

  // Pad remaining bytes with null terminators
  for (size_t i = 2 + portStrLen + 1; i < 8; i++) {
    reqMsg.data[i] = '\0';
  }

  // Set CAN message length to full 8 bytes
  reqMsg.can_dlc = 8;

  int result = sendCanMessage(reqMsg);
  if (result != ERROR_OK) {
    char error_buff[20];
    ReturnErrorString(result, error_buff, 20);
    Serial.printlnf("Failed to send data request to port %d, error: %s", addr,
                    error_buff);
    return false;
  }

  return true;
}
