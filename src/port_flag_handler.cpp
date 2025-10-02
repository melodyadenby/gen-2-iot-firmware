#include "port_flag_handler.h"
#include "can.h"
#include "cloud.h"
#include "port_event_handler.h"
#include "port_state.h"
#include "utils.h"
#include <string.h>

// Global instance declared in main.h, defined in main.ino

PortFlagHandler::PortFlagHandler(PortStateManager *manager)
    : portStateManager(manager), currentPort(1) {
  initializePortMessages();
}

void PortFlagHandler::initializePortMessages() {
  // Pre-build all port data request messages to avoid runtime string operations
  for (int port = 1; port <= MAX_PORTS; port++) {
    can_frame& msg = portDataRequests[port];

    // Clear the entire message
    memset(&msg, 0, sizeof(can_frame));

    // Set the CAN ID
    msg.can_id = port;

    // Set the command prefix
    msg.data[0] = 'R';
    msg.data[1] = ',';

    // Pre-compute the port string ONCE
    if (port < 10) {
      // Single digit port
      msg.data[2] = '0' + port;
      msg.data[3] = '\0';
    } else {
      // Double digit port (10-16)
      msg.data[2] = '1';
      msg.data[3] = '0' + (port - 10);
      msg.data[4] = '\0';
    }

    // Set message length to full 8 bytes (remaining bytes are already zero)
    msg.can_dlc = 8;
  }
}

void PortFlagHandler::processAllPortFlags() {
  // Process flags for all ports in round-robin fashion
  for (int i = 0; i < MAX_PORTS; i++) {
    // Check for cloud messages between each port
    Particle.process();

    int port = getNextPort();
    if (hasPortPendingFlags(port)) {
      processPortFlags(port);

      // Check again after processing a port's flags
      Particle.process();
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

  // Process cloud messages at start of flag processing
  Particle.process();

  // Check for partial VIN timeout
  checkVINTimeout(port, state);

  // logFlagActivity(port, "PROCESSING", "Starting flag processing");

  // Process command flags in priority order
  handleSecurityViolationVINRetry(port);
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

void PortFlagHandler::handleSecurityViolationVINRetry(int port) {
  PortState *state = getPortState(port);
  if (!state || !state->security_violation_retry_active) {
    return;
  }

  unsigned long currentTime = millis();

  // Check if 10 seconds have passed since last retry
  if (currentTime - state->security_vin_retry_timer < 10000) {
    return; // Still waiting for next retry interval
  }

  // Check if VIN is now complete (retry successful)
  if (strlen(state->VIN) >= VIN_LENGTH) {
    Log.info("Port %d - VIN complete during security retry, canceling retry process", port);
    state->security_violation_retry_active = false;
    state->security_vin_retry_count = 0;
    state->security_vin_retry_timer = 0;
    return;
  }

  // Check if we've exhausted all 3 retry attempts
  if (state->security_vin_retry_count >= 3) {
    Log.info("Port %d - Security VIN retries exhausted (%d attempts), triggering EMERGENCY_EXIT",
             port, state->security_vin_retry_count);

    // Clear retry state
    state->security_violation_retry_active = false;
    state->security_vin_retry_count = 0;
    state->security_vin_retry_timer = 0;

    // Now trigger emergency exit
    state->emergency_exit_flag = true;

    Log.info("Port %d - Emergency exit triggered for VIN timeout", port);
    return;
  }

  // Send VIN request
  state->security_vin_retry_count++;
  Log.info("Port %d - Security violation VIN retry %d/3", port, state->security_vin_retry_count);

  logFlagActivity(port, "SECURITY_VIN_RETRY", "Sending VIN request");

  if (sendPortCommand(port, 'K', nullptr, 10 * SEC_TO_MS_MULTIPLIER) == ERROR_OK) {
    state->security_vin_retry_timer = currentTime;
    Log.info("Port %d - Security VIN retry %d sent successfully", port, state->security_vin_retry_count);
  } else {
    handleCommandError(port, "SECURITY_VIN_RETRY", -1);
    // Still update timer to prevent rapid retries on error
    state->security_vin_retry_timer = currentTime;
  }
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
    // Don't reset retry count here - only reset when VIN is complete
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
    Log.info(
        "SECURITY VIOLATION: Cannot charge port %d - not authorized!", port);
    Log.info(
        "Port %d - Charge command blocked to prevent unauthorized charging. "
        "Vehicle will be ejected after grace period if VIN not validated.",
        port);
    state->send_charge_flag = false;
    state->check_charge_status = false;
    return;
  }

  logFlagActivity(port, "CHARGE", "Sending charge command");
  Log.info("Port %d - Charging authorized for secured vehicle with VIN",
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
  Log.info("Status Tag request for port %d\n", port);
  if (sendPortCommand(port, 'S', "0", 10 * SEC_TO_MS_MULTIPLIER) == ERROR_OK) {
    Log.info("Tag data requested successfully");
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
    Log.info("Failed to send port params!");
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
    // Clear all VIN cloud flags BEFORE sending - prevents race conditions
    state->send_vin_to_cloud_flag = false;
    state->awaiting_cloud_vin_resp = false;
    state->cloud_vin_resp_timer = 0;

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "C,2,%d,%s", port, state->VIN);
    publishToCloud(buffer);

    Log.info("Port %d - VIN sent to cloud (one-time only): %s", port,
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

    // Clear flags BEFORE publishing
    state->check_heartbeat_status = false;
    state->heartbeat_success = false;

    // Publish heartbeat success to cloud: H,0,port,1
    char buffer[16];
    formatCloudMessage("H", "0", port, "1", buffer, sizeof(buffer));
    publishToCloud(buffer);
  } else if (state->command_timeout <= 0) {
    logFlagActivity(port, "HEARTBEAT", "Timeout");

    // Clear flag BEFORE publishing
    state->check_heartbeat_status = false;

    // Publish heartbeat failure to cloud: H,0,port,0
    char buffer[16];
    formatCloudMessage("H", "0", port, "0", buffer, sizeof(buffer));
    publishToCloud(buffer);
  }
}

void PortFlagHandler::handleUnlockSuccess(int port) {
  logFlagActivity(port, "UNLOCK", "Success");

  // Reset retry count and clear flags BEFORE publishing
  PortState *state = getPortState(port);
  if (state) {
    state->unlock_retry_count = 0;
    state->check_unlock_status = false;
    state->unlock_successful = false;
  }

  char buffer[16];
  formatCloudMessage("U", "0", port, "1", buffer, sizeof(buffer));
  publishToCloud(buffer);

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

  // Reset all unlock-related flags BEFORE publishing
  state->check_unlock_status = false;
  state->unlock_retry_count = 0;

  char buffer[16];
  formatCloudMessage("U", "0", port, "0", buffer, sizeof(buffer));
  publishToCloud(buffer);
}

void PortFlagHandler::handleChargeSuccess(int port) {
  logFlagActivity(port, "CHARGE", "Success");

  PortState *state = getPortState(port);
  if (state) {
    // // Publish charge success to cloud: C,variant,port,1
    char buffer[16];
    formatCloudMessage("C", "1", port, "1", buffer, sizeof(buffer));
     publishToCloud(buffer);

    state->check_charge_status = false;
    state->charge_successful = false;
    state->charge_varient = '\0';
  }
}

void PortFlagHandler::handleChargeFailure(int port) {
  logFlagActivity(port, "CHARGE", "Failed - timeout");

  PortState *state = getPortState(port);
  if (state) {
    // // Publish charge failure to cloud: C,variant,port,0
    char buffer[16];
    formatCloudMessage("C", "0", port, "0", buffer, sizeof(buffer));
    publishToCloud(buffer);

    state->check_charge_status = false;
    state->charge_successful = false;
    state->charge_varient = '\0';
  }
}

int PortFlagHandler::sendPortCommand(int port, char command,
                                     const char *variant, int timeout) {
  Log.info("GOING TO SEND MESSAGE FOR PORT %d", port);
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
  publishJuiseMessage(message);
}

void PortFlagHandler::resetPortAfterOperation(int port) {
  PortState *state = getPortState(port);
  if (state) {
    state->check_unlock_status = false;
    state->emergency_exit_flag = false;
    state->unlock_successful = false;
    state->send_vin_to_cloud_flag = false;
    state->send_vin_request_timer = 0;
    state->vin_request_flag = false;

    // Clear VIN cloud response state
    state->awaiting_cloud_vin_resp = false;
    state->cloud_vin_resp_timer = 0;

    // Clear VIN and charge variant only on unlock (vehicle leaving)
    memset(state->VIN, 0, sizeof(state->VIN));
    state->charge_varient = '\0';

    Log.info("Port %d - Port state reset after operation, VIN cleared",
                    port);
  }
}

void PortFlagHandler::logFlagActivity(int port, const char *flagName,
                                      const char *action) {
  Log.info("Port %d - %s: %s", port, flagName, action);
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
  Log.info("CAN ERROR on port %d for command %s (error: %d)", port,
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
  const unsigned long VIN_RETRY_BACKOFF = 5000; // 5 second backoff between retries

  // Only check if we have a partial VIN
  if (strlen(state->VIN) > 0 && strlen(state->VIN) < VIN_LENGTH) {
    unsigned long timeSinceLastUpdate =
        millis() - state->send_vin_request_timer;
    if (timeSinceLastUpdate > VIN_TIMEOUT) {
      Log.info("Port %d - Partial VIN timeout after %lu ms (retry count: %d)",
                      port, timeSinceLastUpdate, state->vin_retry_count);

      // Add exponential backoff to prevent rapid retries
      unsigned long backoffDelay = VIN_RETRY_BACKOFF * (state->vin_retry_count + 1);
      if (backoffDelay > 60000) backoffDelay = 60000; // Cap at 60 seconds

      // Check if enough time has passed since last retry
      if (timeSinceLastUpdate < (VIN_TIMEOUT + backoffDelay)) {
        // Still in backoff period, don't retry yet
        return;
      }

      // Clear partial VIN and retry with backoff
      memset(state->VIN, 0, sizeof(state->VIN));
      state->vin_request_flag = true;
      state->send_vin_request_timer = millis();
      state->vin_retry_count++; // Increment retry counter

      Log.info("Port %d - VIN timeout recovery: retry %d (next backoff: %lu ms)",
                      port, state->vin_retry_count, backoffDelay);
    }
  }
}

int PortFlagHandler::portWriteParams(int port, char volts[], char amps[],
                                     int timeout) {
  Log.info("Sending charge params - volts: %s, amps: %s\n", volts, amps);

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
    Log.info("CAN Error in portWriteParams: %d", result);
  }
  return result;
}

int PortFlagHandler::portWrite(int port, char cmd, char *variant, int timeout) {
  struct PortState *portState = getPortState(port);

  Log.info("portWriteNew: port=%d, cmd=%c", port, cmd);

  struct can_frame reqMsg;
  if (port < 1 || port > MAX_PORTS) {
    Log.info("portWrite Invalid port number: %d\n", port);
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
    Log.info("Sending to port: %d\nData: %s\n", port, reqMsg.data);
    // Set the CAN message length to 8
    reqMsg.can_dlc = 8;
  }

  // Attempt to send the message
  int result = sendCanMessage(reqMsg);

  // Check for send errors
  if (result != ERROR_OK) {
    Log.info("CAN send error: %d when sending to port %d", result, port);
  }

  return result;
}

bool PortFlagHandler::sendGetPortData(int addr) {
  if (addr == 0) { // Address 0 is reserved for the IoT device itself
    return true;
  }

  if (!isValidPort(addr)) {
    Log.info("Invalid port number for data request: %d", addr);
    return false;
  }

  // Use pre-built message - NO STRING OPERATIONS!
  int result = sendCanMessage(portDataRequests[addr]);
  if (result != ERROR_OK) {
    char error_buff[20];
    ReturnErrorString(result, error_buff, 20);
    Log.info("Failed to send data request to port %d, error: %s", addr,
                    error_buff);
    return false;
  }

  return true;
}
