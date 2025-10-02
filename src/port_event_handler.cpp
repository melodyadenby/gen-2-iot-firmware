#include "port_event_handler.h"
#include "cloud.h"
#include "port_state.h"
#include "utils.h"
#include <string.h>

// Global instance declared in main.h, defined in main.ino

PortEventHandler::PortEventHandler(PortStateManager *manager)
    : portStateManager(manager) {}

void PortEventHandler::handleCANMessage(const ParsedCANMessage &message) {
  if (!isValidPortNumber(message.sourcePort)) {
    Log.info("Invalid port number in CAN message: %d", message.sourcePort);
    return;
  }

  logMessageProcessing(message);

  switch (message.messageType) {
  case CAN_MSG_STATUS:
    handleStatusMessage(message);
    break;
  case CAN_MSG_VIN: {
    // Check if this is a spontaneous VIN (not requested AND VIN buffer already
    // full)
    PortState *state = getPortState(message.sourcePort);
    if (state && !state->vin_request_flag && strlen(state->VIN) >= VIN_LENGTH) {
      // Spontaneous VIN with full buffer - port is starting new session
      handleSpontaneousVINMessage(message);
    } else {
      // Requested VIN or partial VIN continuation - normal response
      handleVINMessage(message);
    }
  } break;

  case CAN_MSG_TEMPERATURE:
    handleTemperatureMessage(message);
    break;

  case CAN_MSG_FIRMWARE:
    handleFirmwareMessage(message);
    break;

  case CAN_MSG_HEARTBEAT:
    handleHeartbeatMessage(message);
    break;

  case CAN_MSG_UNLOCK:
    handleUnlockMessage(message);
    break;

  case CAN_MSG_CHARGE:
    handleChargeMessage(message);
    break;

  case CAN_MSG_FORCE_EJECT:
    handleForceEjectMessage(message);
    break;

  default:
    Log.info("Unknown CAN message type received from port %d",
             message.sourcePort);
    break;
  }
}

void PortEventHandler::handleStatusMessage(const ParsedCANMessage &message) {
  int port = message.sourcePort;

  // Update port state based on status message
  PortState *state = getPortState(port);
  if (state) {
    state->docked = message.status.docked;
    state->charging = message.status.charging;
    state->vehicle_secured = message.status.vehicleSecured;
    state->valid_vehicle_tag = message.status.tagValid;
    state->fatal_NFC_error = message.status.fatalNFCError;
    markPortPolled(port);
  }

  // SECURITY CHECK: Detect unauthorized charging with grace period for
  // authentication
  if (message.status.docked && message.status.charging &&
      !message.status.vehicleSecured) {
    // Vehicle is charging but not secured - this is always a violation
    Log.info("SECURITY VIOLATION: Port %d - Vehicle charging without "
             "security verification!",
             port);
    Log.info("Port %d - Docked: %s, Charging: %s, Secured: %s, VIN: '%s'", port,
             message.status.docked ? "YES" : "NO",
             message.status.charging ? "YES" : "NO",
             message.status.vehicleSecured ? "YES" : "NO", state->VIN);

    // Immediately trigger emergency exit
    state->emergency_exit_flag = true;

    // Notify cloud of security violation
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "SECURITY_VIOLATION,%d,CHARGING_UNSECURED",
             port);
    publishStatusToCloud(port, buffer, sizeof(buffer));

    Log.info("Port %d - Emergency exit triggered for unsecured charging", port);
    return; // Skip further processing for this compromised port
  }

  // SECURITY CHECK: Detect charging with incomplete VIN after grace period
  if (message.status.docked && message.status.charging &&
      message.status.vehicleSecured && strlen(state->VIN) < VIN_LENGTH) {

    // Check if we're in the authentication grace period (30 seconds)
    const unsigned long VIN_AUTH_GRACE_PERIOD = 30000; // 30 seconds
    unsigned long timeSinceVINRequest = 0;

    // Initialize timer if not set (e.g., IoT restart with vehicle already
    // charging)
    if (state->send_vin_request_timer == 0) {
      state->send_vin_request_timer = millis();
      Log.info("Port %d - Starting grace period timer for charging "
               "vehicle without VIN",
               port);
    }

    if (state->send_vin_request_timer > 0) {
      timeSinceVINRequest = millis() - state->send_vin_request_timer;
    }

    if (timeSinceVINRequest > VIN_AUTH_GRACE_PERIOD) {
      Log.info("SECURITY VIOLATION: Port %d - Charging with incomplete "
               "VIN after grace period!",
               port);
      Log.info("Port %d - VIN length: %d, Grace period expired: %lu ms", port,
               strlen(state->VIN), timeSinceVINRequest);

      // Start security violation VIN retry process (3 attempts, 10s apart)
      if (!state->security_violation_retry_active) {
        Log.info("Port %d - Starting security violation VIN retry process (3 "
                 "attempts, 10s apart)",
                 port);
        state->security_violation_retry_active = true;
        state->security_vin_retry_count = 0;
        state->security_vin_retry_timer = millis();

        // Notify cloud of security violation
        char buffer[64];
        snprintf(buffer, sizeof(buffer),
                 "SECURITY_VIOLATION,%d,INCOMPLETE_VIN_TIMEOUT", port);
        publishStatusToCloud(port, buffer, sizeof(buffer));
      }
      return;
    } else {
      // Still within grace period - log but allow continued operation
      Log.info("Port %d - Charging with incomplete VIN, within grace "
               "period (%lu/%lu ms) - VIN length: %d",
               port, timeSinceVINRequest, VIN_AUTH_GRACE_PERIOD,
               strlen(state->VIN));

      // Request VIN if we haven't already
      if (!state->vin_request_flag) {
        state->vin_request_flag = true;
        Log.info("Port %d - Requesting VIN during grace period", port);
      }
    }
  }

  // SECURITY CHECK: Detect trapped vehicles without VIN after grace period
  // (regardless of charging status)
  if (message.status.docked && message.status.vehicleSecured &&
      strlen(state->VIN) < VIN_LENGTH && !message.status.charging) {

    // Check if we're in the authentication grace period (30 seconds)
    const unsigned long VIN_AUTH_GRACE_PERIOD = 30000; // 30 seconds
    unsigned long timeSinceVINRequest = 0;

    // Initialize timer if not set (e.g., IoT restart with vehicle already
    // docked)
    if (state->send_vin_request_timer == 0) {
      state->send_vin_request_timer = millis();
      Log.info("Port %d - Starting grace period timer for trapped "
               "vehicle without VIN",
               port);
    }

    if (state->send_vin_request_timer > 0) {
      timeSinceVINRequest = millis() - state->send_vin_request_timer;
    }

    if (timeSinceVINRequest > VIN_AUTH_GRACE_PERIOD) {
      Log.info("SECURITY VIOLATION: Port %d - Vehicle trapped without "
               "VIN after grace period!",
               port);
      Log.info("Port %d - VIN length: %d, Grace period expired: %lu ms", port,
               strlen(state->VIN), timeSinceVINRequest);

      // Start security violation VIN retry process (3 attempts, 10s apart)
      if (!state->security_violation_retry_active) {
        Log.info("Port %d - Starting security violation VIN retry process for "
                 "trapped vehicle",
                 port);
        state->security_violation_retry_active = true;
        state->security_vin_retry_count = 0;
        state->security_vin_retry_timer = millis();

        // Notify cloud of security violation
        char buffer[64];
        snprintf(buffer, sizeof(buffer),
                 "SECURITY_VIOLATION,%d,TRAPPED_VIN_TIMEOUT", port);
        publishStatusToCloud(port, buffer, sizeof(buffer));
      }
      return;
    } else {
      // Still within grace period - ensure VIN is being requested
      if (!state->vin_request_flag) {
        state->vin_request_flag = true;
        Log.info("Port %d - Requesting VIN for trapped vehicle during "
                 "grace period",
                 port);
      }
    }
  }

  // Clear VIN if vehicle is no longer docked (handles undocking without unlock)
  if (!message.status.docked && strlen(state->VIN) > 0) {
    Log.info("Port %d - Vehicle undocked, clearing VIN (%s)", port, state->VIN);
    memset(state->VIN, 0, sizeof(state->VIN));
    state->vin_request_flag = false;
    state->send_vin_to_cloud_flag = false;
    state->awaiting_cloud_vin_resp = false;
    state->cloud_vin_resp_timer = 0;

    // Clear security violation retry state
    state->security_violation_retry_active = false;
    state->security_vin_retry_count = 0;
    state->security_vin_retry_timer = 0;

    //one time unlock to reset vars
    char buffer[16];
    formatCloudMessage("U", "0", port, "1", buffer, sizeof(buffer));
    publishJuiseMessage(buffer);
  }

  // Detect port restart scenario: charging stopped but vehicle still secured
  if (strlen(state->VIN) > 0 && !message.status.charging &&
      message.status.docked && message.status.vehicleSecured &&
      !state->awaiting_cloud_vin_resp) {
    Log.info("Port %d - Port restart detected: charging stopped but "
             "vehicle secured and we are not awaiting the cloud",
             port);
    Log.info("Port %d - Clearing VIN state to restart session", port);

    // Clear VIN state to restart the session
    memset(state->VIN, 0, sizeof(state->VIN));
    state->vin_request_flag = false;
    state->send_vin_to_cloud_flag = false;
    state->awaiting_cloud_vin_resp = false;
    state->cloud_vin_resp_timer = 0;

    // Reset charging-related flags
    state->check_charge_status = false;
    state->charge_successful = false;
    state->send_charge_flag = false;
    state->charge_varient = '\0';

    Log.info("Port %d - VIN state cleared, will restart sequence", port);
  }

  // Handle VIN request logic - start VIN sequence for vehicles without VIN
  if (message.status.tagValid && message.status.docked &&
      !state->vin_request_flag && strlen(state->VIN) == 0) {
    // Start VIN sequence if we don't already have a VIN
    state->vin_request_flag = true;
    state->send_vin_request_timer = millis();
    Log.info("Port %d - Starting new VIN sequence", port);
  }

  // Special case: Vehicle is secured and charging but we have no VIN (IoT
  // restart scenario)
  if (message.status.tagValid && message.status.docked &&
      message.status.charging && strlen(state->VIN) == 0 &&
      !state->vin_request_flag) {
    state->vin_request_flag = true;
    state->send_vin_request_timer = millis();
    Log.info("Port %d - Emergency VIN request for charging vehicle without VIN",
             port);
  }

  // Handle charging logic
  if (message.status.charging) {
    state->charging = true;
    // Set charge_successful when port reports charging = '1'
    if (state->check_charge_status) {
      state->charge_successful = true;
      Log.info("Port %d - Charging detected via status message", port);
    }
  }

  // Handle vehicle secured state
  if (message.status.vehicleSecured) {
    state->vehicle_secured = true;
  }

  // Handle fatal NFC errors
  if (message.status.fatalNFCError) {
    Log.info("FATAL NFC ERROR on port %d", port);
    // char buffer[32];
    // snprintf(buffer, sizeof(buffer), "NFC_ERROR,%d", port);
    // publishStatusToCloud(port, buffer);
  }
}

void PortEventHandler::handleSpontaneousVINMessage(
    const ParsedCANMessage &message) {
  int port = message.sourcePort;
  PortState *state = getPortState(port);
  if (!state) {
    return;
  }

  Log.info(
      "Port %d - Spontaneous VIN with full buffer, resetting session state",
      port);

  // Clear existing VIN and charging state (new session starting)
  memset(state->VIN, 0, sizeof(state->VIN));
  state->charging = false;
  state->charge_successful = false;
  state->check_charge_status = false;
  state->send_charge_flag = false;
  state->awaiting_cloud_vin_resp = false;
  state->cloud_vin_resp_timer = 0;
  state->vin_request_flag = false;

  // Process the VIN message normally
  handleVINMessage(message);

  // But always send to cloud regardless of charging state
  if (strlen(state->VIN) >= VIN_LENGTH) {
    Log.info("Port %d - Forcing cloud transmission for spontaneous VIN", port);
    state->send_vin_to_cloud_flag = true;
    state->awaiting_cloud_vin_resp = true;
    state->cloud_vin_resp_timer = millis();
  }
}

void PortEventHandler::handleVINMessage(const ParsedCANMessage &message) {
  int port = message.sourcePort;
  PortState *state = getPortState(port);
  if (!state) {
    return;
  }

  // Check for partial VIN timeout (30 seconds)
  const unsigned long VIN_TIMEOUT = 30000;
  if (strlen(state->VIN) > 0 && strlen(state->VIN) < VIN_LENGTH) {
    unsigned long timeSinceLastChunk = millis() - state->send_vin_request_timer;
    if (timeSinceLastChunk > VIN_TIMEOUT) {
      Log.info("Port %d - Partial VIN timeout, clearing and restarting", port);
      memset(state->VIN, 0, sizeof(state->VIN));
      state->vin_request_flag = true; // Request VIN again
      state->send_vin_request_timer = millis();
    }
  }

  // If this is the first chunk of a new VIN sequence, clear the buffer
  if (strlen(state->VIN) == 0) {
    Log.info("Port %d - Starting VIN collection", port);
    state->send_vin_request_timer =
        millis(); // Update timer for timeout tracking
  }

  // Log.info("=== VIN PROCESSING DEBUG ===");
  // Log.info("Port %d - Raw chunk received: '%s'", port,
  //                 message.vinData.vin);
  // Log.info("Port %d - Current VIN before: '%s' (len=%d)", port,
  //                 state->VIN, strlen(state->VIN));

  // Concatenate VIN chunks instead of overwriting
  size_t currentLen = strlen(state->VIN);
  size_t chunkLen = strlen(message.vinData.vin);
  size_t availableSpace =
      sizeof(state->VIN) - currentLen - 1; // -1 for null terminator

  // Log.info("Port %d - Chunk length: %d, Available space: %d", port,
  // chunkLen, availableSpace);

  // Only append if there's space in the VIN buffer
  if (chunkLen <= availableSpace) {
    strcat(state->VIN, message.vinData.vin);
    // Log.info("Port %d - Concatenation successful", port);
  } else {
    // Log.info("Port %d - WARNING: Not enough space for chunk!", port);
  }

  // Log.info("Port %d - VIN after concatenation: '%s' (len=%d)", port,
  // state->VIN, strlen(state->VIN));

  // Check if VIN is complete (16 characters for full VIN)
  if (strlen(state->VIN) >= VIN_LENGTH) {
    Log.info("Port %d - COMPLETE VIN: %s", port, state->VIN);

    state->vin_request_flag = false;
    state->vin_retry_count = 0; // Reset retry counter on successful completion

    // Clear security violation retry state since VIN is now complete
    if (state->security_violation_retry_active) {
      Log.info(
          "Port %d - VIN complete, canceling security violation retry process",
          port);
      state->security_violation_retry_active = false;
      state->security_vin_retry_count = 0;
      state->security_vin_retry_timer = 0;
    }

    // Only send to cloud if vehicle is not already charging (prevents cloud
    // spam on system restart) - unless this is a spontaneous VIN
    Log.info("Port %d - Setting flags for cloud transmission", port);
    state->send_vin_to_cloud_flag = true;
    state->awaiting_cloud_vin_resp = true;
    state->cloud_vin_resp_timer = millis();
    if (!state->charging) {
      // Log.info("Port %d - Setting flags for cloud transmission",
      // port); state->send_vin_to_cloud_flag = true;
      // state->awaiting_cloud_vin_resp = true;
      // state->cloud_vin_resp_timer = millis();
    } else {
      Log.info("Port %d - VIN collected but skipping cloud transmission "
               "(already charging)",
               port);
    }
  } else {
    // Log.info("Port %d - PARTIAL VIN: %s (%d/%d chars)", port,
    // state->VIN,
    //                 strlen(state->VIN), VIN_LENGTH);
  }
  // Log.info("=== END VIN DEBUG ===");
}

void PortEventHandler::handleTemperatureMessage(
    const ParsedCANMessage &message) {
  int port = message.sourcePort;

  // Update temperature data
  setPortTemperature(port, message.tempData.temperature);

  PortState *state = getPortState(port);
  if (state) {
    state->send_temp_req_flag = false;

    // Copy fan speed if available
    if (strlen(message.tempData.fanSpeed) > 0) {
      strncpy(state->fan_speed, message.tempData.fanSpeed,
              sizeof(state->fan_speed) - 1);
      state->fan_speed[sizeof(state->fan_speed) - 1] = '\0';
    }
  }

  Log.info("Temperature data from port %d: %s°C", port,
           message.tempData.temperature);

  // Publish temperature to cloud
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "TEMP,%d,%s", port,
           message.tempData.temperature);
  publishStatusToCloud(port, buffer, sizeof(buffer));
}

void PortEventHandler::handleFirmwareMessage(const ParsedCANMessage &message) {
  int port = message.sourcePort;

  // Update firmware version
  setPortFirmwareVersion(port, message.firmwareData.version);

  PortState *state = getPortState(port);
  if (state) {
    state->send_port_build_version_flag = false;
  }

  Log.info("Firmware version from port %d: %s", port,
           message.firmwareData.version);

  // Publish firmware version to cloud
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "FW_VER,%d,%s", port,
           message.firmwareData.version);
  publishStatusToCloud(port, buffer, sizeof(buffer));
}

void PortEventHandler::handleHeartbeatMessage(const ParsedCANMessage &message) {
  int port = message.sourcePort;

  PortState *state = getPortState(port);
  if (state) {
    state->heartbeat_success = true;
    state->send_port_heartbeat = false;
    state->check_heartbeat_status = true;
  }

  Log.info("Heartbeat response from port %d", port);
}

void PortEventHandler::handleUnlockMessage(const ParsedCANMessage &message) {
  int port = message.sourcePort;

  PortState *state = getPortState(port);
  if (state) {
    state->unlock_successful = true;
    state->send_unlock_flag = false;
    state->emergency_exit_flag = false;
    if (state->VIN[0] != '\0') {
      // if a latch breaks in a field, this check prevents a spam
      state->check_unlock_status =
          true; // enables a send to cloud of a successful unlock
    }
  }

  Log.info("Unlock response from port %d", port);
}

void PortEventHandler::handleChargeMessage(const ParsedCANMessage &message) {
  int port = message.sourcePort;

  PortState *state = getPortState(port);
  if (state) {
    Log.info("⚡ Charge response from port %d, Status: %s", port,
             message.chargeData.status == '1' ? "Charging" : "Not Charging");
    Log.info("⚡ chargeData: %c", message.chargeData.status);
    // Validate charge variant matches what we sent
    if (message.chargeData.status == '1' ) {
      state->charge_successful = true;
      state->charging = true;
      state->check_charge_status = false;
    } else if (message.chargeData.status == '0') {
      state->charging = false;
      state->charge_successful = false;
      state->check_charge_status = true;
    } else {
      Log.info("Charge status unknown on port %d. Expected: %c, Got: %c",
               port, message.chargeData.status);
    }

    state->send_charge_flag = false;
  }
}

void PortEventHandler::handleForceEjectMessage(
    const ParsedCANMessage &message) {
  int port = message.sourcePort;

  Log.info("FORCE EJECT INITIATED on port %d", port);

  PortState *state = getPortState(port);
  if (state) {
    state->emergency_exit_flag = true;
    state->send_unlock_flag = true;
  }

  // Publish emergency eject to cloud
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "FORCE_EJECT,%d", port);
  publishStatusToCloud(port, buffer, sizeof(buffer));
}

void PortEventHandler::publishStatusToCloud(int port, const char *status,
                                            size_t statusSize) {
  char eventName[32];
  snprintf(eventName, sizeof(eventName), "port_%d_status", port);

  // Create safe copy with guaranteed null termination
  char safeStatus[256];
  size_t maxCopy = (statusSize > 0 && statusSize < sizeof(safeStatus))
                       ? statusSize - 1
                       : sizeof(safeStatus) - 1;
  strncpy(safeStatus, status, maxCopy);
  safeStatus[maxCopy] = '\0';

  Particle.publish(eventName, safeStatus, PRIVATE);
}

void PortEventHandler::resetPortAfterUnlock(int port) {
  PortState *state = getPortState(port);
  if (state) {
    state->check_unlock_status = false;
    state->emergency_exit_flag = false;
    state->unlock_successful = false;
    state->send_vin_to_cloud_flag = false;
    state->send_vin_request_timer = 0;
    state->vin_request_flag = false;

    // Clear VIN and charge variant only on unlock (vehicle leaving)
    memset(state->VIN, 0, sizeof(state->VIN));
    state->charge_varient = '\0';
    Log.info("Port %d - VIN cleared after unlock (vehicle leaving)", port);
  }
}

void PortEventHandler::resetPortAfterCharge(int port) {
  PortState *state = getPortState(port);
  if (state) {
    state->check_charge_status = false;
    state->charge_successful = false;
    state->charge_varient = '\0';
  }
}

void PortEventHandler::handleCommandTimeout(int port, const char *commandType) {
  Log.info("Command timeout on port %d for command: %s", port, commandType);

  // Publish timeout to cloud
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "TIMEOUT,%d,%s", port, commandType);
  publishStatusToCloud(port, buffer, sizeof(buffer));

  // Reset relevant flags
  PortState *state = getPortState(port);
  if (state) {
    if (strcmp(commandType, "unlock") == 0) {
      state->check_unlock_status = false;
    } else if (strcmp(commandType, "charge") == 0) {
      state->check_charge_status = false;
      state->charge_varient = '\0';
    } else if (strcmp(commandType, "heartbeat") == 0) {
      state->check_heartbeat_status = false;
    }
  }
}

void PortEventHandler::logMessageProcessing(const ParsedCANMessage &message) {
  Log.info("Processing %s message from port %d",
           canProcessor.getMessageTypeString(message.messageType),
           message.sourcePort);
}

bool PortEventHandler::isValidPortNumber(int port) {
  return (port >= 1 && port <= MAX_PORTS);
}

void PortEventHandler::formatCloudMessage(const char *command,
                                          const char *variant, int port,
                                          const char *success, char *buffer,
                                          size_t bufferSize) {
  snprintf(buffer, bufferSize, "%s,%s,%d,%s", command, variant, port, success);
}

bool PortEventHandler::isChargingAuthorized(int port) {
  PortState *state = getPortState(port);
  if (!state) {
    Log.info("Port %d - No state available for charging authorization check",
             port);
    return false;
  }

  // Check if vehicle is properly docked and secured
  if (!state->docked || !state->vehicle_secured) {
    Log.info("Port %d - Charging not authorized: docked=%s, secured=%s", port,
             state->docked ? "YES" : "NO",
             state->vehicle_secured ? "YES" : "NO");
    return false;
  }

  // Check if we have a complete VIN
  if (strlen(state->VIN) < VIN_LENGTH) {
    // Check if we're in the authentication grace period (30 seconds)
    const unsigned long VIN_AUTH_GRACE_PERIOD = 30000; // 30 seconds
    unsigned long timeSinceVINRequest = 0;

    if (state->send_vin_request_timer > 0) {
      timeSinceVINRequest = millis() - state->send_vin_request_timer;
    }

    if (timeSinceVINRequest <= VIN_AUTH_GRACE_PERIOD &&
        state->send_vin_request_timer > 0) {
      Log.info("Port %d - Charging authorized during VIN grace period: "
               "VIN length %d, time elapsed %lu ms",
               port, strlen(state->VIN), timeSinceVINRequest);
      return true; // Allow charging during grace period
    }

    Log.info("Port %d - Charging not authorized: incomplete VIN (length "
             "%d): '%s', grace period expired",
             port, strlen(state->VIN), state->VIN);
    return false;
  }

  // Check if we're awaiting cloud response (charging should only start after
  // cloud approval unless in grace period)
  if (state->awaiting_cloud_vin_resp) {
    Log.info(
        "Port %d - Charging not authorized: still awaiting cloud VIN response",
        port);
    return false;
  }

  Log.info("Port %d - Charging authorized: vehicle secured with valid VIN: %s",
           port, state->VIN);

  return true;
}

void PortEventHandler::logSecurityEvent(int port, const char *eventType,
                                        const char *details) {
  Log.info("SECURITY EVENT - Port %d: %s - %s", port, eventType, details);

  // Publish to cloud for centralized monitoring
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "SECURITY_EVENT,%d,%s,%s", port, eventType,
           details);
  publishStatusToCloud(port, buffer, sizeof(buffer));
}

void PortEventHandler::validatePortSecurity(int port) {
  PortState *state = getPortState(port);
  if (!state) {
    return;
  }

  // Periodic security validation
  if (state->charging) {
    if (!state->vehicle_secured) {
      logSecurityEvent(port, "UNSECURED_CHARGING",
                       "Vehicle charging without security");
      state->emergency_exit_flag = true;
    }

    if (strlen(state->VIN) < VIN_LENGTH) {
      char details[64];
      snprintf(details, sizeof(details), "Incomplete_VIN_length_%d",
               strlen(state->VIN));
      logSecurityEvent(port, "INVALID_VIN_CHARGING", details);
      state->emergency_exit_flag = true;
    }
  }
}
