#ifndef PORT_FLAG_HANDLER_H
#define PORT_FLAG_HANDLER_H

#include "Arduino.h"
#include "Particle.h"
#include "port_state.h"

// Forward declarations
class PortStateManager;

/**
 * Centralized handler for all port flags and commands
 * Replaces the scattered flag processing throughout main.ino
 */
class PortFlagHandler {
private:
  PortStateManager *portStateManager;

public:
  /**
   * Constructor
   * @param manager Pointer to port state manager
   */
  explicit PortFlagHandler(PortStateManager *manager);

  /**
   * Process all pending flags for all ports
   * This is the main entry point called from the main loop
   */
  void processAllPortFlags();

  /**
   * Process flags for a specific port
   * @param port Port number to process
   */
  void processPortFlags(int port);

  /**
   * Handle security violation VIN retry process
   * @param port Port number
   */
  void handleSecurityViolationVINRetry(int port);

  /**
   * Handle VIN request flag
   * @param port Port number
   */
  void handleVINRequest(int port);

  /**
   * Handle unlock command flag
   * @param port Port number
   */
  void handleUnlockCommand(int port);

  /**
   * Handle manual tag read flag
   * @param port Port number
   */
  void handleManualTagRead(int port);

  /**
   * Handle charge command flag
   * @param port Port number
   */
  void handleChargeCommand(int port);

  /**
   * Handle heartbeat flag
   * @param port Port number
   */
  void handleHeartbeat(int port);

  /**
   * Handle temperature request flag
   * @param port Port number
   */
  void handleTemperatureRequest(int port);

  /**
   * Handle charging parameters flag
   * @param port Port number
   */
  void handleChargingParameters(int port);

  /**
   * Handle port build version request flag
   * @param port Port number
   */
  void handlePortVersionRequest(int port);

  /**
   * Handle emergency exit flag
   * @param port Port number
   */
  void handleEmergencyExit(int port);

  /**
   * Handle VIN to cloud flag
   * @param port Port number
   */
  void handleVINToCloud(int port);

  /**
   * Handle button state flag
   * @param port Port number
   */
  void handleButtonState(int port);

  /**
   * Check and handle command timeouts
   * @param port Port number
   */
  void checkCommandTimeouts(int port);

  /**
   * Check unlock status after command
   * @param port Port number
   */
  void checkUnlockStatus(int port);

  /**
   * Check charge status after command
   * @param port Port number
   */
  void checkChargeStatus(int port);

  /**
   * Check heartbeat status after command
   * @param port Port number
   */
  void checkHeartbeatStatus(int port);

  /**
   * Handle successful unlock completion
   * @param port Port number
   */
  void handleUnlockSuccess(int port);

  /**
   * Handle failed unlock completion
   * @param port Port number
   */
  void handleUnlockFailure(int port);

  /**
   * Handle successful charge completion
   * @param port Port number
   */
  void handleChargeSuccess(int port);

  /**
   * Handle failed charge completion
   * @param port Port number
   */
  void handleChargeFailure(int port);

  /**
   * Send CAN command to port
   * @param port Target port number
   * @param command Command character
   * @param variant Optional variant parameter
   * @param timeout Command timeout in seconds
   * @return Error code (ERROR_OK on success)
   */
  int sendPortCommand(int port, char command, const char *variant, int timeout);

  /**
   * Send charging parameters to port
   * @param port Target port number
   * @param volts Voltage parameter
   * @param amps Amperage parameter
   * @param timeout Command timeout
   * @return Error code (ERROR_OK on success)
   */
  int sendChargingParams(int port, const char *volts, const char *amps,
                         int timeout);

  /**
   * Publish message to cloud
   * @param message Message to publish
   */
  void publishToCloud(const char *message);

  /**
   * Reset port state after operation completion
   * @param port Port number
   */
  void resetPortAfterOperation(int port);

  /**
   * Log flag processing activity
   * @param port Port number
   * @param flagName Name of the flag being processed
   * @param action Action being taken
   */
  void logFlagActivity(int port, const char *flagName, const char *action);

  /**
   * Get the next port to process in round-robin fashion
   * @return Next port number to process
   */
  int getNextPort();

  /**
   * Check if port has any pending flags
   * @param port Port number to check
   * @return true if port has pending flags
   */
  bool hasPortPendingFlags(int port);

  /**
   * Get count of ports with pending flags
   * @return Number of ports with pending operations
   */
  int getPendingPortsCount();

  /**
   * Send Information Request To Port
   * Sends "R,<port>" command to request port data
   * @param addr CAN address
   */
  bool sendGetPortData(int addr);

private:
  int currentPort; // Current port being processed in round-robin

  /**
   * Validate port number
   * @param port Port number to validate
   * @return true if valid
   */
  bool isValidPort(int port);

  /**
   * Create formatted cloud message
   * @param command Command type
   * @param success Success flag (0 or 1)
   * @param port Port number
   * @param data Additional data
   * @param buffer Output buffer
   * @param bufferSize Buffer size
   */
  void formatCloudMessage(const char *command, const char *variant, int port,
                          const char *success, char *buffer, size_t bufferSize);

  /**
   * Handle command error
   * @param port Port number
   * @param command Command that failed
   * @param errorCode Error code returned
   */
  void handleCommandError(int port, const char *command, int errorCode);

  /**
   * Check for partial VIN timeout and restart VIN request if needed
   * @param port Port number
   * @param state Port state pointer
   */
  void checkVINTimeout(int port, PortState *state);

  /**
   * Check if a command can be retried based on timing
   * @param lastAttemptTime Last attempt timestamp
   * @param retryInterval Retry interval in milliseconds
   * @return true if retry is allowed
   */
  bool canRetryCommand(unsigned long lastAttemptTime,
                       unsigned long retryInterval);

  /**
   * Update command timeout counter (DEPRECATED - no longer used)
   * Timeouts now use absolute timestamps instead of countdown
   * @param port Port number
   * @param decrement Amount to decrement timeout
   */
  void updateCommandTimeout(int port, int decrement);
  /**
   * Write over CAN to port vehicle charging parameters
   * @param port Port number
   * @param message command character
   * @param message varient character
   * @param desired message timeout
   */
  int portWrite(int port, char cmd, char *variant, int timeout);
  /**
   * Write over CAN to port vehicle charging parameters
   * @param port Port number
   * @param vehicle rated voltage
   * @param vehicle rated amperage
   * @param desired message timeout
   */
  int portWriteParams(int port, char volts[], char amps[], int timeout);
};

// Global instance (extern declaration)
extern PortFlagHandler *portFlagHandler;

#endif // PORT_FLAG_HANDLER_H
