#include "port_state.h"
#include "Particle.h"
#include "config.h"
#include "cloud.h"
#include "utils.h"

// Global port state variables
int CURRENT_PORT = 1;
struct PortState ports[MAX_PORTS];

// IoT State Variables
bool send_signal_flag = false;
bool send_heartbeat_flag = false;
bool send_iot_build_version_flag = false;

void initializePorts() {
  // Initialize all ports to default state
  for (int i = 1; i < MAX_PORTS; i++) {
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
void markPortsUnpolled() {
  Serial.println("Marking ports as unpolled");
  for (int port = 1; port <= MAX_PORTS; port++) {
    PortState *state = &ports[port];
    if (state) {
      state->DID_PORT_CHECK = false;
    }
  }
}

void resetPortState(int portNumber) {
  if (!isValidPort(portNumber)) {
    Serial.printlnf("resetPortState Invalid port number: %d\n", portNumber);
    return;
  }

  int index = portNumber - 1; // Convert to 0-based index
  struct PortState *port = &ports[index];

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
  port->last_emergency_unlock_time = 0;

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

void updatePortState(int portNumber, const struct PortState *newState) {
  if (!isValidPort(portNumber) || newState == NULL) {
    Serial.printlnf("Invalid port number or null state: %d\n", portNumber);
    return;
  }

  int index = portNumber - 1; // Convert to 0-based index
  memcpy(&ports[index], newState, sizeof(struct PortState));

  // Update last poll time
  ports[index].last_poll_time = millis();

  Serial.printlnf("Updated port %d state\n", portNumber);
}

struct PortState *getPortState(int portNumber) {
  if (!isValidPort(portNumber)) {
    Serial.printlnf("getPortState Invalid port number: %d\n", portNumber);
    return NULL;
  }

  int index = portNumber - 1; // Convert to 0-based index
  return &ports[index];
}

bool isValidPort(int portNumber) {
  return (portNumber >= 1 && portNumber <= MAX_PORTS);
}

void setCurrentPort(int portNumber) {
  if (isValidPort(portNumber)) {
    CURRENT_PORT = portNumber;
    Serial.printlnf("Current port set to: %d\n", CURRENT_PORT);
  } else {
    Serial.printlnf("Invalid port number for current port: %d\n", portNumber);
  }
}

int getCurrentPort() { return CURRENT_PORT; }

bool isPortDocked(int portNumber) {
  struct PortState *port = getPortState(portNumber);
  return port ? port->docked : false;
}

bool isPortCharging(int portNumber) {
  struct PortState *port = getPortState(portNumber);
  return port ? port->charging : false;
}

bool isPortSecured(int portNumber) {
  struct PortState *port = getPortState(portNumber);
  return port ? port->vehicle_secured : false;
}

void setPortVIN(int portNumber, const char *vin) {
  struct PortState *port = getPortState(portNumber);
  if (port && vin) {
    safeStrCopy(port->VIN, vin, sizeof(port->VIN));
    Serial.printlnf("Port %d VIN set to: %s\n", portNumber, port->VIN);
  }
}

const char *getPortVIN(int portNumber) {
  struct PortState *port = getPortState(portNumber);
  return port ? port->VIN : "";
}

void setPortTemperature(int portNumber, const char *temperature) {
  struct PortState *port = getPortState(portNumber);
  if (port && temperature) {
    safeStrCopy(port->temp, temperature, sizeof(port->temp));
    Serial.printlnf("Port %d temperature set to: %s\n", portNumber, port->temp);
  }
}

void setPortVoltage(int portNumber, const char *voltage) {
  struct PortState *port = getPortState(portNumber);
  if (port && voltage) {
    safeStrCopy(port->volts, voltage, sizeof(port->volts));
    Serial.printlnf("Port %d voltage set to: %s\n", portNumber, port->volts);
  }
}

void setPortCurrent(int portNumber, const char *current) {
  struct PortState *port = getPortState(portNumber);
  if (port && current) {
    safeStrCopy(port->amps, current, sizeof(port->amps));
    Serial.printlnf("Port %d current set to: %s\n", portNumber, port->amps);
  }
}

void setPortFirmwareVersion(int portNumber, const char *version) {
  struct PortState *port = getPortState(portNumber);
  if (port && version) {
    safeStrCopy(port->port_firmware_version, version,
                sizeof(port->port_firmware_version));
    Serial.printlnf("Port %d firmware version set to: %s\n", portNumber,
                    port->port_firmware_version);
  }
}

String getPortStatusSummary(int portNumber) {
  struct PortState *port = getPortState(portNumber);
  if (!port) {
    return "Invalid port";
  }

  String status = "Port " + String(portNumber) + ": ";

  status += port->fatal_NFC_error ? "Fatal NFC Error, " : "NFC Healthy, ";

  status += port->docked ? "Docked" : "Empty";

  if (port->docked) {
    status += port->charging ? ", Charging" : ", Not Charging";
    status += port->vehicle_secured ? ", Secured" : ", Unsecured";
    status += port->valid_vehicle_tag ? ", Tag Valid" : ", Tag Invalid";

    if (strlen(port->VIN) > 0) {
      status += ", VIN: " + String(port->VIN);
    }
  }

  return status;
}

unsigned long getPortLastPollTime(int portNumber) {
  struct PortState *port = getPortState(portNumber);
  return port ? port->last_poll_time : 0;
}

void markPortPolled(int portNumber) {
  struct PortState *port = getPortState(portNumber);
  if (port) {
    port->last_poll_time = millis();
    port->DID_PORT_CHECK = true;
  }
  // Port status request will be handled by timing mechanism in main loop
  // instead of immediate response
}

bool hasPortBeenPolled(int portNumber) {
  struct PortState *port = getPortState(portNumber);
  return port ? port->DID_PORT_CHECK : false;
}

void clearPortFlags(int portNumber) {
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

char *getPortStatusRange(int startPort, int endPort) {
  static char buffer[256]; // Static buffer to return
  memset(buffer, 0, sizeof(buffer));

  if (startPort < 1 || endPort > MAX_PORTS || startPort > endPort) {
    Serial.printlnf("Invalid port range: %d-%d", startPort, endPort);
    snprintf(buffer, sizeof(buffer), "P,0,ERROR");
    return buffer;
  }

  // Start building the response: P,0,status1,status2,status3,...
  strcpy(buffer, "P,0");

  for (int port = startPort; port <= endPort; port++) {
    struct PortState *portState = getPortState(port);
    char portStatus[8];

    if (portState) {
      // Format: just the status value (1=docked, 0=empty)
      snprintf(portStatus, sizeof(portStatus), ",%d",
               portState->docked ? 1 : 0);
    } else {
      // Invalid port - use 0
      snprintf(portStatus, sizeof(portStatus), ",0");
    }

    // Check if adding this would exceed buffer size
    if (strlen(buffer) + strlen(portStatus) < sizeof(buffer) - 1) {
      strcat(buffer, portStatus);
    } else {
      Serial.println("Port status buffer overflow prevented");
      break;
    }
  }

  Serial.printlnf("Port status range %d-%d: %s", startPort, endPort, buffer);
  return buffer;
}
