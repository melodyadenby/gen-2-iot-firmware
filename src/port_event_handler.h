#ifndef PORT_EVENT_HANDLER_H
#define PORT_EVENT_HANDLER_H

#include "Arduino.h"
#include "Particle.h"
#include "can_processor.h"
#include "port_state.h"

// Forward declarations
class PortStateManager;

/**
 * Handles business logic for port events triggered by CAN messages
 * Separates CAN message parsing from port state management
 */
class PortEventHandler {
private:
  PortStateManager *portStateManager;

public:
  /**
   * Constructor
   * @param manager Pointer to port state manager
   */
  explicit PortEventHandler(PortStateManager *manager);

  /**
   * Process a parsed CAN message and apply business logic
   * @param message Parsed CAN message from CANMessageProcessor
   */
  void handleCANMessage(const ParsedCANMessage &message);

  /**
   * Handle status update messages (D)
   * @param message Parsed status message
   */
  void handleStatusMessage(const ParsedCANMessage &message);

  /**
   * Handle VIN response messages (K)
   * @param message Parsed VIN message
   */
  void handleVINMessage(const ParsedCANMessage &message);

  /**
   * Handle spontaneous VIN messages (K) that reset session state
   * @param message Parsed VIN message
   */
  void handleSpontaneousVINMessage(const ParsedCANMessage &message);

  /**
   * Handle temperature data messages (T)
   * @param message Parsed temperature message
   */
  void handleTemperatureMessage(const ParsedCANMessage &message);

  /**
   * Handle firmware version messages (V)
   * @param message Parsed firmware message
   */
  void handleFirmwareMessage(const ParsedCANMessage &message);

  /**
   * Handle heartbeat response messages (H)
   * @param message Parsed heartbeat message
   */
  void handleHeartbeatMessage(const ParsedCANMessage &message);

  /**
   * Handle unlock response messages (U)
   * @param message Parsed unlock message
   */
  void handleUnlockMessage(const ParsedCANMessage &message);

  /**
   * Handle charge response messages (C)
   * @param message Parsed charge message
   */
  void handleChargeMessage(const ParsedCANMessage &message);

  /**
   * Handle force eject messages (F)
   * @param message Parsed force eject message
   */
  void handleForceEjectMessage(const ParsedCANMessage &message);

  /**
   * Publish status update to cloud
   * @param port Port number
   * @param status Status message to publish
   */
  void publishStatusToCloud(int port, const char *status);

  /**
   * Reset port state after successful unlock
   * @param port Port number
   */
  void resetPortAfterUnlock(int port);

  /**
   * Reset port state after charge operation
   * @param port Port number
   */
  void resetPortAfterCharge(int port);

  /**
   * Handle command timeout scenarios
   * @param port Port number
   * @param commandType Type of command that timed out
   */
  void handleCommandTimeout(int port, const char *commandType);

  /**
   * Log message processing for debugging
   * @param message Parsed message to log
   */
  void logMessageProcessing(const ParsedCANMessage &message);

private:
  /**
   * Validate port number is in acceptable range
   * @param port Port number to validate
   * @return true if valid, false otherwise
   */
  bool isValidPortNumber(int port);

  /**
   * Create formatted status string for cloud publishing
   * @param command Command type
   * @param variant Command variant
   * @param port Port number
   * @param success Success flag ("0" or "1")
   * @param buffer Output buffer
   * @param bufferSize Size of output buffer
   */
  void formatCloudMessage(const char *command, const char *variant, int port,
                          const char *success, char *buffer, size_t bufferSize);
};

// Global instance (extern declaration)
extern PortEventHandler *portEventHandler;

#endif // PORT_EVENT_HANDLER_H
