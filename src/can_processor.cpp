#include "can_processor.h"
#include "config.h"
#include <string.h>

// Global instance
CANMessageProcessor canProcessor;

ParsedCANMessage
CANMessageProcessor::parseMessage(const can_frame &rawMessage) {
  ParsedCANMessage parsedMsg;

  // Initialize with defaults
  parsedMsg.sourcePort = rawMessage.can_id;
  parsedMsg.payloadLength = rawMessage.can_dlc;
  parsedMsg.isValid = false;
  memset(&parsedMsg.payload, 0, sizeof(parsedMsg.payload));

  // Validate basic message structure
  if (!isValidMessage(rawMessage)) {
    parsedMsg.messageType = CAN_MSG_UNKNOWN;
    return parsedMsg;
  }

  // Copy payload
  memcpy(parsedMsg.payload, rawMessage.data, rawMessage.can_dlc);

  // Determine message type from first byte
  parsedMsg.messageType = static_cast<CANMessageType>(rawMessage.data[0]);

  // Parse based on message type
  switch (parsedMsg.messageType) {
  case CAN_MSG_STATUS:
    parseStatusMessage(rawMessage.data, rawMessage.can_dlc, parsedMsg);
    break;

  case CAN_MSG_VIN:
    parseVINMessage(rawMessage.data, rawMessage.can_dlc, parsedMsg);
    break;

  case CAN_MSG_TEMPERATURE:
    parseTemperatureMessage(rawMessage.data, rawMessage.can_dlc, parsedMsg);
    break;

  case CAN_MSG_FIRMWARE:
    parseFirmwareMessage(rawMessage.data, rawMessage.can_dlc, parsedMsg);
    break;

  case CAN_MSG_CHARGE:
    parseChargeMessage(rawMessage.data, rawMessage.can_dlc, parsedMsg);
    break;

  case CAN_MSG_HEARTBEAT:
  case CAN_MSG_UNLOCK:
  case CAN_MSG_FORCE_EJECT:
    // Simple messages with no additional parsing needed
    parsedMsg.isValid = true;
    break;

  default:
    parsedMsg.messageType = CAN_MSG_UNKNOWN;
    parsedMsg.isValid = false;
    break;
  }

  return parsedMsg;
}

bool CANMessageProcessor::isValidMessage(const can_frame &rawMessage) {
  // Check port validity
  if (!isValidPort(rawMessage.can_id)) {
    return false;
  }

  // Check message length
  if (rawMessage.can_dlc == 0 || rawMessage.can_dlc > 8) {
    return false;
  }

  // Check if first byte is a valid message type
  char msgType = rawMessage.data[0];
  return (msgType == 'D' || msgType == 'K' || msgType == 'T' ||
          msgType == 'F' || msgType == 'V' || msgType == 'H' ||
          msgType == 'U' || msgType == 'C');
}

const char *CANMessageProcessor::getMessageTypeString(CANMessageType type) {
  switch (type) {
  case CAN_MSG_STATUS:
    return "STATUS";
  case CAN_MSG_VIN:
    return "VIN";
  case CAN_MSG_TEMPERATURE:
    return "TEMPERATURE";
  case CAN_MSG_FORCE_EJECT:
    return "FORCE_EJECT";
  case CAN_MSG_FIRMWARE:
    return "FIRMWARE";
  case CAN_MSG_HEARTBEAT:
    return "HEARTBEAT";
  case CAN_MSG_UNLOCK:
    return "UNLOCK";
  case CAN_MSG_CHARGE:
    return "CHARGE";
  default:
    return "UNKNOWN";
  }
}

bool CANMessageProcessor::isValidPort(uint32_t port) {
  return (port >= 1 && port <= MAX_PORTS);
}

void CANMessageProcessor::parseStatusMessage(const uint8_t *payload,
                                             size_t length,
                                             ParsedCANMessage &parsedMsg) {
  if (length < 6) {
    parsedMsg.isValid = false;
    return;
  }

  // Parse status fields:
  // D[tag_valid][docked][vehicle_secured][charge_status][fatal_nfc_error]
  parsedMsg.status.tagValid = (payload[1] == '1');
  parsedMsg.status.docked = (payload[2] == '1');
  parsedMsg.status.vehicleSecured = (payload[3] == '1');
  parsedMsg.status.charging = (payload[4] == '1');
  parsedMsg.status.fatalNFCError = (payload[5] == '1');

  parsedMsg.isValid = true;
}

void CANMessageProcessor::parseVINMessage(const uint8_t *payload, size_t length,
                                          ParsedCANMessage &parsedMsg) {
  if (length < 2) {
    parsedMsg.isValid = false;
    return;
  }

  // Debug: Print raw payload
  Serial.print("VIN Raw payload: ");
  for (size_t i = 0; i < length; i++) {
    Serial.printf("%c", payload[i]);
  }
  Serial.println();

  Serial.print("VIN Raw bytes: ");
  for (size_t i = 0; i < length; i++) {
    Serial.printf("0x%02X ", payload[i]);
  }
  Serial.println();

  // Check if payload starts with 'K'
  if (payload[0] != 'K') {
    Serial.println("VIN message doesn't start with 'K'");
    parsedMsg.isValid = false;
    return;
  }

  // Extract VIN data after "K" prefix (no comma expected)
  size_t vinDataLen = length - 1; // Skip only the 'K'
  if (vinDataLen >= sizeof(parsedMsg.vinData.vin)) {
    vinDataLen = sizeof(parsedMsg.vinData.vin) - 1;
  }

  safeExtractString(payload, 1, vinDataLen, parsedMsg.vinData.vin,
                    sizeof(parsedMsg.vinData.vin));

  Serial.printf("Extracted VIN chunk: '%s'\n", parsedMsg.vinData.vin);
  parsedMsg.isValid = true;
}

void CANMessageProcessor::parseTemperatureMessage(const uint8_t *payload,
                                                  size_t length,
                                                  ParsedCANMessage &parsedMsg) {
  if (length < 3) {
    parsedMsg.isValid = false;
    return;
  }

  // Extract temperature data after "T," prefix
  size_t tempDataLen = length - 2;
  if (tempDataLen >= sizeof(parsedMsg.tempData.temperature)) {
    tempDataLen = sizeof(parsedMsg.tempData.temperature) - 1;
  }

  safeExtractString(payload, 2, tempDataLen, parsedMsg.tempData.temperature,
                    sizeof(parsedMsg.tempData.temperature));

  // For now, fan speed is not included in the message format
  memset(parsedMsg.tempData.fanSpeed, 0, sizeof(parsedMsg.tempData.fanSpeed));

  parsedMsg.isValid = true;
}

void CANMessageProcessor::parseFirmwareMessage(const uint8_t *payload,
                                               size_t length,
                                               ParsedCANMessage &parsedMsg) {
  if (length < 3) {
    parsedMsg.isValid = false;
    return;
  }

  // Extract firmware version after "V," prefix
  size_t versionDataLen = length - 2;
  if (versionDataLen >= sizeof(parsedMsg.firmwareData.version)) {
    versionDataLen = sizeof(parsedMsg.firmwareData.version) - 1;
  }

  safeExtractString(payload, 2, versionDataLen, parsedMsg.firmwareData.version,
                    sizeof(parsedMsg.firmwareData.version));

  parsedMsg.isValid = true;
}

void CANMessageProcessor::parseChargeMessage(const uint8_t *payload,
                                             size_t length,
                                             ParsedCANMessage &parsedMsg) {
  if (length < 3) {
    parsedMsg.isValid = false;
    return;
  }

  // Extract charge variant from position 2: C,[variant]
  parsedMsg.chargeData.variant = payload[2];

  parsedMsg.isValid = true;
}

void CANMessageProcessor::safeExtractString(const uint8_t *payload,
                                            size_t offset, size_t length,
                                            char *output, size_t outputSize) {
  // Ensure we don't read beyond payload or output buffer
  size_t copyLen = length;
  if (copyLen >= outputSize) {
    copyLen = outputSize - 1;
  }

  // Copy the data
  memcpy(output, payload + offset, copyLen);

  // Null terminate
  output[copyLen] = '\0';
}
