#include "port_flag_handler.h"
#include "can.h"
#include "mqtt.h"
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

  logFlagActivity(port, "PROCESSING", "Starting flag processing");

  // Process command flags in priority order
  handleEmergencyExit(port);
  handleVINRequest(port);
  handleUnlockCommand(port);
  handleChargeCommand(port);
  handleHeartbeat(port);
  handleTemperatureRequest(port);
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

  if (sendPortCommand(port, 'K', nullptr, 10) == ERROR_OK) {
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
    logFlagActivity(port, "EMERGENCY_EXIT", "Processing emergency unlock");

    if (sendPortCommand(port, 'U', "0", 10) == ERROR_OK) {
      state->send_unlock_flag = false;
      state->check_unlock_status = true;
    } else {
      handleCommandError(port, "EMERGENCY_UNLOCK", -1);
    }
  } else if (state->send_unlock_flag) {
    logFlagActivity(port, "UNLOCK", "Sending unlock command");

    if (sendPortCommand(port, 'U', nullptr, 10) == ERROR_OK) {
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

  logFlagActivity(port, "CHARGE", "Sending charge command");

  // Convert single char to string
  char variantStr[2] = {state->charge_varient, '\0'};

  if (sendPortCommand(port, 'C', variantStr, 65) == ERROR_OK) {
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

  if (sendPortCommand(port, 'H', nullptr, 5) == ERROR_OK) {
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

  if (sendPortCommand(port, 'T', "0", 10) == ERROR_OK) {
    state->send_temp_req_flag = false;
  } else {
    handleCommandError(port, "TEMPERATURE", -1);
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

  if (sendPortCommand(port, 'V', nullptr, 10) == ERROR_OK) {
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

  logFlagActivity(port, "EMERGENCY_EXIT", "Processing emergency exit");

  // Emergency exit triggers unlock command
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

    state->send_vin_to_cloud_flag = false;
    state->awaiting_cloud_vin_resp = true;
    state->cloud_vin_resp_timer = millis();
  }
}

void PortFlagHandler::handleButtonState(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->send_button_state_flag) {
    return;
  }

  logFlagActivity(port, "BUTTON_STATE", "Sending button state");

  char buffer[32];
  snprintf(buffer, sizeof(buffer), "BUTTON,%d,%c", port, state->button_state);
  publishToCloud(buffer);

  state->send_button_state_flag = false;
}

void PortFlagHandler::checkCommandTimeouts(int port) {
  PortState *state = getPortState(port);
  if (!state) {
    return;
  }

  if (state->command_timeout > 0) {
    updateCommandTimeout(port, 1);
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

  resetPortAfterOperation(port);
}

void PortFlagHandler::handleUnlockFailure(int port) {
  logFlagActivity(port, "UNLOCK", "Failed - timeout");

  char buffer[16];
  formatCloudMessage("U", "0", port, "0", buffer, sizeof(buffer));
  publishToCloud(buffer);

  PortState *state = getPortState(port);
  if (state) {
    state->check_unlock_status = false;
    state->DID_PORT_CHECK = false;
  }
}

void PortFlagHandler::handleChargeSuccess(int port) {
  logFlagActivity(port, "CHARGE", "Success");

  PortState *state = getPortState(port);
  if (state) {
    // Publish charge success to cloud: C,variant,port,1
    char buffer[16];
    char variantStr[2] = {state->charge_varient, '\0'};
    formatCloudMessage("C", variantStr, port, "1", buffer, sizeof(buffer));
    publishToCloud(buffer);

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
    // Publish charge failure to cloud: C,variant,port,0
    char buffer[16];
    char variantStr[2] = {state->charge_varient, '\0'};
    formatCloudMessage("C", variantStr, port, "0", buffer, sizeof(buffer));
    publishToCloud(buffer);

    state->check_charge_status = false;
    state->DID_PORT_CHECK = false;
    state->charge_varient = '\0';
  }
}

int PortFlagHandler::sendPortCommand(int port, char command,
                                     const char *variant, int timeout) {
  if (!isValidPort(port)) {
    return -1;
  }

  // Use existing portWriteNew function
  return portWriteNew(port, command, const_cast<char *>(variant), timeout);
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

    // Clear VIN and charge variant
    memset(state->VIN, 0, sizeof(state->VIN));
    state->charge_varient = '\0';
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
          state->send_temp_req_flag || state->send_charging_params_flag ||
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

  char buffer[64];
  snprintf(buffer, sizeof(buffer), "CAN_ERROR,%d,%s,%d", port, command,
           errorCode);
  publishToCloud(buffer);
}

bool PortFlagHandler::canRetryCommand(unsigned long lastAttemptTime,
                                      unsigned long retryInterval) {
  return (millis() - lastAttemptTime) >= retryInterval;
}

void PortFlagHandler::updateCommandTimeout(int port, int decrement) {
  PortState *state = getPortState(port);
  if (state && state->command_timeout > 0) {
    state->command_timeout -= decrement;
    if (state->command_timeout < 0) {
      state->command_timeout = 0;
    }
  }
}
int PortFlagHandler::portWriteParams(int port, char volts[], char amps[],
                                     int timeout) {
  Serial.printf("Sending charge params - volts: %s, amps: %s\n", volts, amps);

  struct can_frame reqMsg;
  ports[port].command_timeout = timeout;

  reqMsg.can_id = port;
  reqMsg.can_dlc = 0;

  // Build message: P,volts,amps
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

  return sendCanMessage(reqMsg);
}

int PortFlagHandler::portWriteNew(int port, char cmd, char *variant,
                                  int timeout) {

  struct can_frame reqMsg;
  if (port < 1 || port > MAX_PORTS) {
    Serial.printf("Invalid port number: %d\n", port);
    return -1;
  }

  // Set command timeout
  ports[port].command_timeout = timeout;

  // Build CAN message
  reqMsg.can_id = port;
  reqMsg.can_dlc = 0;

  // Add command
  reqMsg.data[reqMsg.can_dlc++] = cmd;

  // Add comma separator
  reqMsg.data[reqMsg.can_dlc++] = ',';

  // Add variant if provided
  if (variant && strlen(variant) > 0) {
    for (int i = 0; i < strlen(variant) && reqMsg.can_dlc < 8; i++) {
      reqMsg.data[reqMsg.can_dlc++] = variant[i];
    }
  } else {
    reqMsg.data[reqMsg.can_dlc++] = '0';
  }

  // Send the message
  int result = sendCanMessage(reqMsg);

  if (result == ERROR_OK) {
    Serial.printlnf("Command '%c' sent to port %d successfully", cmd, port);
  } else {
    Serial.printlnf("Failed to send command '%c' to port %d (error: %d)", cmd,
                    port, result);
  }

  return result;
}
