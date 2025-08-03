#ifndef PORT_STATE_H
#define PORT_STATE_H

#include "Arduino.h"
#include "Particle.h"
#include "config.h"

// Message queue constants
#define MAX_QUEUED_MESSAGES_PER_PORT 5

// Structure for queued messages
struct QueuedMessage
{
  char command;
  char variant[8]; // Enough space for variants like "0", "1", etc.
  int timeout;
  bool hasVariant;
  unsigned long queueTime; // When this message was queued
};

const unsigned long PORT_CHECK_INTERVAL = 20 * SEC_TO_MS_MULTIPLIER;
const unsigned long POLL_STAGGER_DELAY = 300; // 500ms between requests
const unsigned long SUBSEQUENT_MSG_DELAY =
    2000; // 1000ms delay between messages to same port

const int VIN_LENGTH = 16;
const int MAX_UNLOCK_RETRY = 3;
// Port State Structure
struct PortState
{
  // Message queue for this port
  QueuedMessage messageQueue[MAX_QUEUED_MESSAGES_PER_PORT];
  int queueHead;  // Index of next message to send
  int queueTail;  // Index where to add next message
  int queueCount; // Number of messages in queue
  bool DID_PORT_CHECK = false;
  bool docked;            // Is port docked
  bool charging;          // Is port charging
  bool valid_vehicle_tag; // Tag was successfully read from adapter
  bool
      vehicle_secured;           // Port is breathing yellow, latch is shut, tag was read.
  int command_timeout;           // Absolute timestamp (millis()) when command should
                                 // timeout
  char button_state;             // Button state
  char VIN[17];                  // Current VIN
  bool vin_request_flag = false; // Flag to call VIN
  unsigned long send_vin_request_timer;
  unsigned long last_poll_time = 0; // Last time this port was polled
  unsigned long last_message_time =
      0;                             // Last time a message was sent to this port
  bool send_button_state_flag;       // Flag to send button state
  bool emergency_exit_flag = false;  // Flag to eject vehicle
  bool send_port_build_version_flag; // Flag to send port version no.
  bool send_temp_req_flag;           // Flag to send request for temperature data
  bool send_charging_params_flag;    // Flag to send port charge params
  bool send_charge_flag;             // Flag to send charge command
  bool send_unlock_flag;             // Flag to send unlock command
  bool send_vin_to_cloud_flag;
  bool awaiting_cloud_vin_resp; // Flag that we are awaiting a vehicle valid
                                // response from the cloud
  unsigned long
      cloud_vin_resp_timer;    // timer of last call to cloud for VIN response
  bool send_port_heartbeat;    // Flag to send port heartbeat command
  bool check_heartbeat_status; // Flag to check unlock status
  bool check_unlock_status;    // Flag to check unlock status
  bool check_charge_status;    // Flag to check charge status
  bool unlock_successful;      // unlock success?
  bool charge_successful;      // charge success?
  bool heartbeat_success;
  int unlock_retry_count;        // Number of unlock retries attempted
  char charge_varient;           // Current charge varient
  char volts[3];                 // Port Voltage
  char amps[3];                  // Port Amperage
  bool fatal_NFC_error;          // WEEWOO
  char temp[8];                  // port temperature in celsius
  char fan_speed[4];             // PWM fan speed
  char port_firmware_version[9]; // firmware version of the port
};

// Global Port State
extern int CURRENT_PORT;
extern struct PortState ports[MAX_PORTS];

// IoT State Variables
extern bool send_signal_flag;            // flag to send signal string
extern bool send_heartbeat_flag;         // flag to send signal string
extern bool send_iot_build_version_flag; // flag to send signal string

// Port Management Functions
void initializePorts();
void resetPortState(int portNumber);
void updatePortState(int portNumber, const struct PortState *newState);
struct PortState *getPortState(int portNumber);
bool isValidPort(int portNumber);
void setCurrentPort(int portNumber);
int getCurrentPort();

// Port State Query Functions
bool isPortDocked(int portNumber);
bool isPortCharging(int portNumber);
bool isPortSecured(int portNumber);
const char *getPortVIN(int portNumber);
String getPortStatusSummary(int portNumber);
char *getPortStatusRange(int startPort, int endPort);
unsigned long getPortLastPollTime(int portNumber);
bool hasPortBeenPolled(int portNumber);

// Port State Setter Functions
void setPortVIN(int portNumber, const char *vin);
void setPortTemperature(int portNumber, const char *temperature);
void setPortVoltage(int portNumber, const char *voltage);
void setPortCurrent(int portNumber, const char *current);
void setPortFirmwareVersion(int portNumber, const char *version);
void markPortPolled(int portNumber);
void clearPortFlags(int portNumber);
void markPortsUnpolled();

// Message delay management functions
bool canSendMessageToPort(int portNumber);
void markMessageSentToPort(int portNumber);
unsigned long getTimeSinceLastMessage(int portNumber);

// Message queue management functions
bool queueMessageForPort(int portNumber, char command, const char *variant,
                         int timeout);
bool hasQueuedMessages(int portNumber);
bool sendNextQueuedMessage(int portNumber);
void processPortMessageQueues();
int getQueuedMessageCount(int portNumber);
void clearPortMessageQueue(int portNumber);

#endif // PORT_STATE_H
