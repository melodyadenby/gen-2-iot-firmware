
#include "Arduino.h"
#include "Particle.h"
#include "fixes/json_compat.h"
#include "lights.h"
#include "utils.h"
#include <ArduinoJson.h>
#include <MQTT.h>
#include "can.h"
#include "main.h"

// ApplicationWatchdog *wd;
MQTT client(MQTT_URL, MQTT_PORT, MQTT_MAX_PACKET_SIZE, KEEP_ALIVE,
            mqtt_callback);
void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    delay(2000);
    Serial.printlnf("*** KUHMUTE IoT V %s ***\n", BUILD_VERSION);
    Serial.printlnf("Device ID: %s\n", Particle.deviceID().c_str());

    Particle.function("resetDevice", resetDevice);

    // setup ringlight
    beginLight();
    setLightBlue();

    // setup CAN
    // Initialize SPI explicitly (required for MCP2515 communication)
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV8);

    int err = mcp2515.reset();
    if (err != mcp2515.ERROR_OK)
        reportCANError(err, "reset", true);

    err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
    if (err != mcp2515.ERROR_OK)
        reportCANError(err, "setBitrate", true);

    delay(50);
    err = mcp2515.setNormalMode();
    if (err != mcp2515.ERROR_OK)
        reportCANError(err, "setNormalMode", true);

    pinMode(CAN_INT, INPUT_PULLUP);
    attachInterrupt(CAN_INT, can_interrupt, FALLING);

    // setup Particle
    Particle.connect();
    Particle.publishVitals();
    Particle.keepAlive(23 * 60); // send a ping every 23 minutes

    System.enableFeature(FEATURE_RESET_INFO);
    System.enableFeature(FEATURE_RETAINED_MEMORY);

    // Initialize Particle Watchdog (60 second timeout)
    // wd = new ApplicationWatchdog(60000, watchdogTimeoutCallback, 1536);

    // Gets MQTT Credentials once connected to Particle Cloud
    Particle.subscribe(PARTICLE_CREDENTIALS_SUBSCRIBE, getIotCredentials,
                       MY_DEVICES);

    setLightRed();
    waitFor(Particle.connected, 60000);
    if (!Particle.connected())
    {
        resetDevice(" ");
    }

    System.enableFeature(FEATURE_RESET_INFO);
    int reason = System.resetReason();
    Serial.println("🚨 RESET REASON ");
    Serial.println(reason);
    int reasonData = System.resetReasonData();
    Serial.println("🚨 RESET REASON DATA ");
    Serial.println(reasonData);

    char buffer[100];
    snprintf(buffer, 100, "Reason: %d, Data: %d", reason, reasonData);
    puts(buffer);
    Particle.publish("RESET REASON", buffer, PRIVATE);

    setLightBlue();

    Serial.println("*** Calling credentials in setUp ");
    Particle.publish(PARTICLE_CREDENTIALS, deviceIdBuf, PRIVATE);
    attemptedCredentialsFetchCount++;
    last_credentials_call = millis();
}

void loop()
{
    if (CAN_ERROR)
    {
        blinkCANError();
    }
    else if (attemptedCredentialsFetchCount > 10)
    {
        Serial.println("Failed to fetch credentials after 10 attempts");
        blinkIdentityError();
    }
    else if (Particle.connected())
    {
        handleMQTTClientLoop();
        // Checks to make only AFTER internet connection established
        if (!credentialsFetched)
        {
            setLightBlue();
            // Do not communicate to ports until credentials for MQTT broker are
            // validated.
            long time = millis();
            if (time - last_credentials_call > 10 * SEC_TO_MS_MULTIPLIER &&
                attemptedCredentialsFetchCount <= 10)
            {
                Serial.println("*** Calling credentials in loop ");
                Particle.publish(PARTICLE_CREDENTIALS, deviceIdBuf, PRIVATE);
                attemptedCredentialsFetchCount++;
                last_credentials_call = millis();
            }
        }
        else if (BROKER_CONNECTED)
        {
            // Connected to internet AND MQTT is connected
            setLightGreen();
        }
    }
    else
    {
        setLightRed();
    }
}
void can_interrupt()
{
    // Minimize work in the interrupt handler to prevent lockups
    struct can_frame recMsg;

    // Disable interrupts during critical section
    noInterrupts();

    // Just read the message and add to queue - defer all processing to main
    // loop
    int readin = readCanMessage(&recMsg);

    if (readin == ERROR_OK)
    {
        Serial.println("CAN message received");
        Serial.printf("%s", recMsg.data);
        Serial.println();
    }
    else if (readin == MCP2515::ERROR_FAIL)
    {
        // Possible bus error - count it but don't act in interrupt
        Serial.println("CAN message error");
        // message_error_count++;
    }

    // Clear interrupt flags immediately
    clearCanInterrupts();

    // Re-enable interrupts
    interrupts();
}
void reportCANError(int err, const char *operation, bool report)
{
    if (report)
    {
        CAN_ERROR = err != 0;
    }
    if (err != 0)
    {
        // printf("Current date and time: %s\n", Time.day());
        char ret[150];
        Serial.print("CAN BUS ERROR ");
        Serial.println(operation);
        ReturnErrorString(err, ret, sizeof(ret));
        sprintf(can_err_msg, "CAN BUS ERROR %s: %s", operation, ret);

        // sprintf(can_err_msg, "CAN BUS ERROR %s (%s): %s", operation,
        // Time.day(), ret);
        puts(can_err_msg);
        // Particle.publish("CAN ERROR", can_err_msg, PRIVATE);
    }
}
/*
 * * MQTT LOGIC
 */
void handleMQTTClientLoop()
{
    if (!client.loop())
    {
        Serial.println("client loop failed");
    }
    BROKER_CONNECTED = client.isConnected();

    checkMQTTStat();
}

void checkMQTTStat()
{
    if (!credentialsFetched)
    {
        // Don't attempt reconnection until credentials are fetched
        return;
    }

    if (BROKER_CONNECTED)
    {
        resetRetryLogic();
        return;
    }
    unsigned long current_time = millis();
    // MQTT is disconnected
    if (!mqtt_disconnect_noted)
        mqtt_disconnected_timer = millis();

    // Attempt reconnection faster
    if (current_time - mqtt_disconnected_timer >= MQTT_DISCONNECTED_TIMEOUT)
    {
        attemptReconnect();
    }
}
void attemptReconnect()
{
    unsigned long currentMillis = millis();

    Serial.print("Time since last attempt: ");
    Serial.println(currentMillis - lastRetryTime);
    Serial.print("Current retry interval: ");
    Serial.println(currentRetryInterval);

    if (currentMillis - lastRetryTime >= currentRetryInterval)
    {
        Serial.println("Attempting to reconnect MQTT...");
        if (connect_mqtt())
        {
            Serial.println("Reconnected to MQTT broker.");
            resetRetryLogic();
        }
        else
        {
            Serial.println("Reconnect attempt failed.");
            lastRetryTime = currentMillis; // Update last retry time on attempt
            increaseRetryInterval();
        }
    }
}
void resetRetryLogic()
{
    mqtt_disconnect_noted = false;
    currentRetryInterval = RETRY_INTERVAL_MS; // Reset to initial retry interval
    lastRetryTime = millis();                 // Reset the timer to current time
}
void increaseRetryInterval()
{
    currentRetryInterval *= 2; // Exponential backoff
    if (currentRetryInterval > MAX_RETRY_INTERVAL_MS)
    {
        currentRetryInterval = MAX_RETRY_INTERVAL_MS;
    }
}
void build_topics(const char *base)
{

    if (MANUAL_MODE[0] != '\0')
    {
        snprintf(MQTT_PUB_TOPIC, sizeof(MQTT_PUB_TOPIC), "%s%s%s", topic_base,
                 MANUAL_MODE, PUB_BASE);
        snprintf(MQTT_SUB_TOPIC, sizeof(MQTT_SUB_TOPIC), "%s%s%s", topic_base,
                 MANUAL_MODE, SUB_BASE);
    }
    else
    {
        snprintf(MQTT_PUB_TOPIC, sizeof(MQTT_PUB_TOPIC), "%s%s%s", topic_base, base,
                 PUB_BASE);
        snprintf(MQTT_SUB_TOPIC, sizeof(MQTT_SUB_TOPIC), "%s%s%s", topic_base, base,
                 SUB_BASE);
    }

    Serial.printf("Pub Topic: %s\n", MQTT_PUB_TOPIC);
    Serial.println("");
    Serial.printf("Sub Topic: %s\n", MQTT_SUB_TOPIC);
    Serial.println("");
}

bool connect_mqtt() { return connect_broker(); }

bool connect_broker()
{
    logDebugInfo("Connecting to MQTT broker");
    if (client.isConnected())
    {
        Serial.println("Already connected to MQTT broker.");
        logDebugInfo("Already connected to MQTT");
        return true; // If already connected, don't attempt to reconnect
    }

    if (!Particle.connected())
    {
        Serial.println("Internet not connected. Cannot connect to MQTT.");
        return false;
    }

    bool res = client.connect(MQTT_CLIENT_ID, MQTT_USR, MQTT_PWD, MQTT_PUB_TOPIC,
                              MQTT::QOS1, 5, "A,0,0", true);
    Serial.print("MQTT Connection Status: ");
    Serial.println(res ? "Connected" : "Failed");
    if (res)
    {
        if (lastConnected == 0 || lastConnected >= KEEP_ALIVE * 1000)
        {
            sub_topic_and_alert();
        }
        lastConnected = millis();
    }
    else
    {
        // Log specific error if possible
        Serial.print("MQTT connection failed with error: ");
        // Log error code if available from your MQTT library
    }
    BROKER_CONNECTED = res;
    return res;
}
void sub_topic_and_alert()
{
    Serial.println("PUBLISHING A,0,1");
    int res = client.publish(MQTT_PUB_TOPIC, "A,0,1");
    Serial.print("sub_topic_and_alert: MQTT PUB STAT:");
    Serial.println(res);
    if (!res)
    {
        Serial.println("FAILED TO PUBLISH");
        return;
    }
    res = client.subscribe(MQTT_SUB_TOPIC);
    Serial.print("sub_topic_and_alert: MQTT SUB STAT:");
    Serial.println(res);
    if (!res)
    {
        Serial.println("FAILED TO SUBSCRIBE");
        return;
    }
}

void publishCloud(String message)
{
    Serial.print("Going to publish cloud: ");
    Serial.println(message);

    logDebugInfo("Before MQTT publish");
    bool res = client.publish(MQTT_PUB_TOPIC, message);
    logDebugInfo("After MQTT publish");

    Serial.print("publishCloud result: ");
    Serial.println(res);

    if (res)
    {
        last_mqtt_send = millis(); // Update the last successful send time
        MQTT_FAIL_COUNT = 0;       // Reset failure count after success
    }
    else
    {
        MQTT_FAIL_COUNT++;
        lastHeartbeatRetryTime = millis(); // Store last failed attempt time
    }
}

void getIotCredentials(const char *event, const char *data)
{
    Serial.printf("Received response: %s\n", data);
    Serial.println("");
    Serial.print("This is for device id: ");
    Serial.println(System.deviceID());

    // Create a StaticJsonDocument with an estimation of the size required.
    StaticJsonDocument<1000> jsonDoc;

    // Deserialize the JSON data
    DeserializationError error = deserializeJson(jsonDoc, data);
    if (error)
    {
        Serial.printf("deserializeJson() failed with code %s\n", error.c_str());
        Serial.println("");
        return;
    }

    // Get values from the parsed JSON
    const char *pubId = jsonDoc["pubId"];
    const char *mqttUser = jsonDoc["mqttUser"];

    if (pubId && mqttUser)
    {
        char creds[100];

        // Copy values to global variables
        strncpy(MANUAL_MODE, pubId, sizeof(MANUAL_MODE) - 1);
        MANUAL_MODE[sizeof(MANUAL_MODE) - 1] = '\0'; // Ensure null-terminated
        strncpy(MQTT_CLIENT_ID, Particle.deviceID(), sizeof(MQTT_CLIENT_ID) - 1);
        MQTT_CLIENT_ID[sizeof(MQTT_CLIENT_ID) - 1] = '\0'; // Ensure null-terminated

        // Print values
        snprintf(creds, 100, "PubID: %s, MQTT User: %s\n", MANUAL_MODE,
                 MQTT_CLIENT_ID);
        puts(creds);
        Serial.printf("PubID: %s, MQTT User: %s\n", MANUAL_MODE, MQTT_CLIENT_ID);
        Serial.println("");
        Particle.publish("Got Creds: ", creds, PRIVATE);

        Particle.unsubscribe();
        build_topics(MQTT_USR);
        credentialsFetched = true;
        if (!connect_mqtt())
        {
            Serial.println("Error: connect_mqtt");
            blinkIdentityError();
        }

        // sendSignalString();
    }
    else
    {
        credentialsFetched = false;
        Particle.publish(PARTICLE_CREDENTIALS,
                         "Failed to fetch credentials. JSON malformed.", PRIVATE);
        Serial.println("Error: Missing pubId or mqttUser in the received JSON:");
        Serial.printf("%s", jsonDoc);
        Serial.println("");
        blinkIdentityError();
    }
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = '\0';

    Serial.print("!!!!!!!!!!!!!!!!!!!!!!! Got a message : ");
    Serial.println(p);

    // Parse the message by comma delimiter
    char *token;
    char *rest = p;
    char *tokens[4] = {NULL}; // Store up to 4 tokens (cmd, variant, port, btn)
    int i = 0;

    // Split the string by commas
    while ((token = strtok_r(rest, ",", &rest)) && i < 4)
    {
        tokens[i++] = token;
    }

    // Extract command from first token
    char cmd = tokens[0] ? tokens[0][0] : '\0';

    // Extract variant from second token if available
    char variant = tokens[1] ? tokens[1][0] : '\0';

    // Extract port from third token if available (properly convert string to int)
    int port = 0;
    if (tokens[2])
    {
        port = atoi(tokens[2]);
        Serial.print("Parsed port number: ");
        Serial.println(port);
    }

    // Extract button state from fourth token if available
    char btn = tokens[3] ? (tokens[3][0] == '0' ? '1' : '0') : '2';
    // print_port(port);
    // switch (cmd)
    // {
    // case 'C':
    // {
    //     portState[port].check_charge_status = true;
    //     portState[port].charge_varient = variant;
    //     portState[port].send_charge_flag = true;
    //     portState[port].awaiting_cloud_vin_resp = false;

    //     break;
    // }
    // // do unlock
    // case 'U':
    // {
    //     portState[port].send_unlock_flag = true;

    //     break;
    // }
    // case 'H':
    // {
    //     // Do heartbeat
    //     switch (variant)
    //     {
    //     case '0':
    //         switch (p[4])
    //         {
    //         case '0': // Heartbeat IoT
    //             publishCloud("H,0,1");
    //             break;
    //         default:
    //         {
    //             // Get port number from tokens[2] which should be the port
    //             number if (tokens[2] && *tokens[2])
    //             {
    //                 int port = atoi(tokens[2]);
    //                 if (port >= 1 && port <= MAX_PORTS)
    //                 {
    //                     portState[port].send_port_heartbeat = true;
    //                 }
    //                 else
    //                 {
    //                     Serial.printf("Error: Invalid port number %d. Ignoring
    //                     message.",
    //                                   port);
    //                     Serial.println();
    //                 }
    //             }
    //             else
    //             {
    //                 Serial.println("Error: Missing port number. Ignoring
    //                 message.");
    //             }
    //             break;
    //         }
    //         }
    //         break;
    //     }
    //     break;
    // }
    // case 'V':
    // {
    //     switch (variant)
    //     {
    //     case '0':
    //     {
    //         send_iot_build_version_flag = true;
    //         break;
    //     }
    //     case '1':
    //     {
    //         portState[port].send_port_build_version_flag = true;
    //         break;
    //     }
    //     default:
    //     {
    //         break;
    //     }
    //     }
    //     break;
    // }
    // case 'T':
    // {

    //     portState[port].send_temp_req_flag = true;
    //     break;
    // }
    // case 'S':
    // {
    //     send_signal_flag = true;
    //     break;
    // }
    // case 'E':
    // {
    //     // Emergency exit
    //     switch (variant)
    //     {
    //     case '0':
    //     {
    //         Serial.print("Emergency Exit Flag enabled for port: ");
    //         Serial.println(port);
    //         if (port >= 1 && port <= MAX_PORTS)
    //         {
    //             portState[port].emergency_exit_flag = true;
    //         }
    //         else
    //         {
    //             Serial.println("Invalid port number for emergency exit");
    //         }
    //         break;
    //     }
    //     }
    //     break;
    // }
    // case 'B':
    // {
    //     portState[port].button_state = btn;
    //     portState[port].send_button_state_flag = true;
    //     break;
    // }
    // case 'P': // P,0,<port_start>, <port_end>
    // {
    //     if (length > 10)
    //     {
    //         break; // invalid call, ignore
    //     }
    //     strncpy(portStatusRequest, p, sizeof(portStatusRequest) - 1);
    //     portStatusRequest[sizeof(portStatusRequest) - 1] = '\0';
    //     break;
    // }
    // case 'K': // Kvin check
    // {
    //     // Button
    //     switch (variant)
    //     {
    //         // Vin Invalid
    //     case '0':
    //     {
    //         Serial.print("Emergency Exit Flag enabled for port: ");
    //         Serial.println(port);
    //         if (port >= 1 && port <= MAX_PORTS)
    //         {
    //             portState[port].emergency_exit_flag = true;
    //         }
    //         else
    //         {
    //             Serial.println("Invalid port number for emergency exit (VIN
    //             Invalid)");
    //         }
    //         break;
    //     }
    //         // Vin valid
    //     case '1':
    //     {
    //         Serial.println("VIN Validated, charging allowed");

    //         portState[port].check_charge_status = true;
    //         portState[port].charge_varient = variant;
    //         portState[port].send_charge_flag = true;
    //         portState[port].awaiting_cloud_vin_resp = false;
    //         Serial.println("Printing each character with index:");
    //         for (int i = 0; i < strlen(p); i++)
    //         {
    //             Serial.printf("p[%d]: %c", i, p[i]);
    //             Serial.println("");
    //         }
    //         char volts[3]; // Buffer for voltage with space for null terminator
    //         char amps[3];  // Buffer for amperage with space for null
    //         terminator

    //         // Assuming 'p' is a properly null-terminated string coming as
    //         input volts[0] = p[8]; volts[1] = p[9]; volts[2] = '\0'; //
    //         Null-terminate the string

    //         amps[0] = p[11];
    //         amps[1] = p[12];
    //         amps[2] = '\0'; // Null-terminate the string

    //         Serial.printf("Volts: %s, Amps: %s", volts, amps);
    //         Serial.println();

    //         portState[port].button_state = btn;
    //         portState[port].send_button_state_flag = true;

    //         strcpy(portState[port].volts, volts);
    //         strcpy(portState[port].amps, amps);
    //         portState[port].send_charging_params_flag = true;
    //         break;
    //     }
    //     }
    //     break;
    // }
    // default:
    //     break;
    // }
}

int resetDevice(String command)
{
    // Log reset request with reason if provided
    if (command.length() > 0)
    {
        Serial.printf("Device reset requested. Reason: %s\n", command.c_str());

        // Report last state before reset
        Serial.printf("Free memory before reset: %lu bytes\n", System.freeMemory());
        Serial.printf("System uptime: %lu ms\n", millis());
        Serial.printf("MQTT connected: %s\n", client.isConnected() ? "yes" : "no");

        // Attempt to close connections gracefully
        if (client.isConnected())
        {
            client.disconnect();
            Serial.println("MQTT connection closed");
        }

        delay(1000); // Give serial time to send
    }
    System.reset();
    return 1; // won't actually get here, but good practice to return a value
}
void logDebugInfo(const char *checkpoint)
{
    static unsigned long lastLogTime = 0;
    unsigned long now = millis();
    Serial.printf("[%lu ms] Checkpoint: %s ", now, checkpoint);
    Serial.println(now);
    Serial.printf("(Delta: %lu ms)", checkpoint, now - lastLogTime);
    Serial.println();
    lastLogTime = now;
}
