#ifndef CAN_PROCESSOR_H
#define CAN_PROCESSOR_H

#include "Arduino.h"
#include "Particle.h"
#include "mcp2515.h"

// CAN Message Types
enum CANMessageType {
  CAN_MSG_STATUS = 'D',      // Port status update
  CAN_MSG_VIN = 'K',         // VIN response
  CAN_MSG_TEMPERATURE = 'T', // Temperature data
  CAN_MSG_FORCE_EJECT = 'F', // Force eject command
  CAN_MSG_FIRMWARE = 'V',    // Firmware version
  CAN_MSG_HEARTBEAT = 'H',   // Heartbeat response
  CAN_MSG_UNLOCK = 'U',      // Unlock response
  CAN_MSG_CHARGE = 'C',      // Charge response
  CAN_MSG_UNKNOWN = 0        // Unknown message type
};

// Parsed CAN Message Structure
struct ParsedCANMessage {
  uint32_t sourcePort;
  CANMessageType messageType;
  uint8_t payload[8];
  size_t payloadLength;
  bool isValid;

  // Message-specific parsed data
  union {
    // Status message data (D)
    struct {
      bool tagValid;
      bool docked;
      bool vehicleSecured;
      bool charging;
      bool fatalNFCError;
    } status;

    // VIN message data (K)
    struct {
      char vin[17];
    } vinData;

    // Temperature message data (T)
    struct {
      char temperature[8];
      char fanSpeed[4];
    } tempData;

    // Firmware version data (V)
    struct {
      char version[9];
    } firmwareData;

    // Charge response data (C)
    struct {
      char variant;
    } chargeData;
  };
};

class CANMessageProcessor {
public:
  /**
   * Parse a raw CAN frame into a structured message
   * @param rawMessage Raw CAN frame from hardware
   * @return Parsed message structure
   */
  ParsedCANMessage parseMessage(const can_frame &rawMessage);

  /**
   * Validate if a CAN message is properly formatted
   * @param rawMessage Raw CAN frame to validate
   * @return true if message is valid, false otherwise
   */
  bool isValidMessage(const can_frame &rawMessage);

  /**
   * Get human-readable string for message type
   * @param type Message type enum
   * @return String representation of message type
   */
  const char *getMessageTypeString(CANMessageType type);

  /**
   * Check if port number is valid
   * @param port Port number to validate
   * @return true if port is in valid range (1-MAX_PORTS)
   */
  bool isValidPort(uint32_t port);

private:
  /**
   * Parse status message payload (D)
   * @param payload Raw payload data
   * @param length Payload length
   * @param parsedMsg Output parsed message
   */
  void parseStatusMessage(const uint8_t *payload, size_t length,
                          ParsedCANMessage &parsedMsg);

  /**
   * Parse VIN message payload (K)
   * @param payload Raw payload data
   * @param length Payload length
   * @param parsedMsg Output parsed message
   */
  void parseVINMessage(const uint8_t *payload, size_t length,
                       ParsedCANMessage &parsedMsg);

  /**
   * Parse temperature message payload (T)
   * @param payload Raw payload data
   * @param length Payload length
   * @param parsedMsg Output parsed message
   */
  void parseTemperatureMessage(const uint8_t *payload, size_t length,
                               ParsedCANMessage &parsedMsg);

  /**
   * Parse firmware version message payload (V)
   * @param payload Raw payload data
   * @param length Payload length
   * @param parsedMsg Output parsed message
   */
  void parseFirmwareMessage(const uint8_t *payload, size_t length,
                            ParsedCANMessage &parsedMsg);

  /**
   * Parse charge response message payload (C)
   * @param payload Raw payload data
   * @param length Payload length
   * @param parsedMsg Output parsed message
   */
  void parseChargeMessage(const uint8_t *payload, size_t length,
                          ParsedCANMessage &parsedMsg);

  /**
   * Safely extract string from payload with bounds checking
   * @param payload Source payload
   * @param offset Offset in payload to start extraction
   * @param length Maximum length to extract
   * @param output Output buffer
   * @param outputSize Size of output buffer
   */
  void safeExtractString(const uint8_t *payload, size_t offset, size_t length,
                         char *output, size_t outputSize);
};

// Global instance (extern declaration)
extern CANMessageProcessor canProcessor;

#endif // CAN_PROCESSOR_H
