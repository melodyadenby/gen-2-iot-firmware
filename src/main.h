#ifndef MAIN_H
#define MAIN_H

// System Configuration
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(AUTOMATIC);

// Version and Device Info
PRODUCT_VERSION(36);
#define BUILD_VERSION "Gen 0.2.5"

const int MIN_TO_MS_MULTIPLIER = 60000;
const int SEC_TO_MS_MULTIPLIER = 1000;

#define ENV_PROD 0
#define ENV_QA 1
#define ENV_LOCAL 2

// Set the current environment
#define ENV 0

// Configure based on the environment
#if ENV == ENV_QA
#define MQTT_URL "qa-hub-mqtt-broker-config.kuhmute.net"
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/qa-credentials"
#define PARTICLE_CREDENTIALS "qa-credentials"
#elif ENV == ENV_PROD
#define MQTT_URL "hub-mqtt-broker-config.kuhmute.net"
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/credentials"
#define PARTICLE_CREDENTIALS "credentials"
#elif ENV == ENV_LOCAL
#define MQTT_URL "mqtt://198.111.63.104:1883"
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/qa-credentials"
#define PARTICLE_CREDENTIALS "qa-credentials"
#else
#define MQTT_URL "dev-hub-mqtt-broker-config.kuhmute.net"
#define PARTICLE_CREDENTIALS_SUBSCRIBE "hook-response/credentials"
#define PARTICLE_CREDENTIALS "credentials"
#endif

char deviceIdBuf[100];
bool credentialsFetched = false;
struct IotCredentials
{
    String pubId;
    String mqttUser;
    String hub_uuid;
};

// Network and MQTT Definitions
#define MQTT_USR "sspitler"
#define MQTT_PWD "testpass"
#define MQTT_PORT 1883
#define KEEP_ALIVE 120 // seconds
#define MQTT_MAX_PACKET_SIZE 300
#define OLD_MQTT_URL "615.mqtt.kuhmute.org"
char MQTT_CLIENT_ID[45];
char MQTT_PUB_TOPIC[256];
char MQTT_SUB_TOPIC[256];
char MANUAL_MODE[14];
bool BROKER_CONNECTED = false;

int attemptedCredentialsFetchCount = 0;
long last_credentials_call = 0;

unsigned long mqtt_disconnected_timer = 0;
bool mqtt_disconnect_noted = false;
const unsigned long MQTT_DISCONNECTED_TIMEOUT = 5 * SEC_TO_MS_MULTIPLIER;
const unsigned long RETRY_INTERVAL_MS =
    SEC_TO_MS_MULTIPLIER; // Initial retry interval of 1 second
unsigned long currentRetryInterval = RETRY_INTERVAL_MS;
const unsigned long MAX_RETRY_INTERVAL_MS =
    15 * SEC_TO_MS_MULTIPLIER;

unsigned long lastRetryTime = 0;
unsigned long lastConnected = 0;
unsigned long last_mqtt_send = 0; // Tracks the last time mqtt message was sent
int MQTT_FAIL_COUNT = 0;
unsigned long lastHeartbeatRetryTime = 0;

// MQTT Topics and Base Commands
char *topic_base = "/hub/";
char *SUB_BASE = "/cmd";
char *PUB_BASE = "/msg";

// CAN Bus Definitions
#define CAN_INT A4
#define MAX_PORTS 16
char can_err_msg[200];
bool CAN_ERROR = false;

int CURRENT_PORT = 1;

struct PortState
{
    bool DID_PORT_CHECK = false;
    bool docked;            // Is port docked
    bool charging;          // Is port charging
    bool valid_vehicle_tag; // Tag was successfully read from adapter
    bool
        vehicle_secured;           // Port is breathing yellow, latch is shut, tag was read.
    int command_timeout;           // Timeout for command
    char button_state;             // Button state
    char VIN[17];                  // Current VIN
    bool vin_request_flag = false; // Flag to call VIN
    unsigned long send_vin_request_timer;
    unsigned long last_poll_time = 0;  // Last time this port was polled
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
    char charge_varient;           // Current charge varient
    char volts[3];                 // Port Voltage
    char amps[3];                  // Port Amperage
    bool fatal_NFC_error;          // WEEWOO
    char temp[8];                  // port temperature in celsius
    char fan_speed[4];             // PWM fan speed
    char port_firmware_version[9]; // firmware version of the port
};

// IoT State Variables
bool send_signal_flag;            // flag to send signal string
bool send_heartbeat_flag;         // flag to send signal string
bool send_iot_build_version_flag; // flag to send signal string
char portStatusRequest[10];

int resetDevice(String command);
void logDebugInfo(const char *checkpoint);
int resetDevice(String command);
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void getIotCredentials(const char *event, const char *data);
void publishCloud(String message);
void sub_topic_and_alert();
bool connect_broker();
bool connect_mqtt();
void build_topics(const char *base);

#endif // MAIN_H