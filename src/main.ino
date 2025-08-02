#include "Arduino.h"
#include "Particle.h"
#include "can.h"
#include "config.h"
#include "credentials.h"
#include "fixes/json_compat.h"
#include "lights.h"
#include "main.h"
#include "mqtt.h"
#include "port_state.h"
#include "utils.h"
#include <ArduinoJson.h>
#include <MQTT.h>
#include <SPI.h>

// Product version setup
PRODUCT_VERSION(PRODUCT_VERSION_NUM);

// Global CAN state variables
char can_err_msg[200];
bool CAN_ERROR = false;

void setup()
{
    initializeSystem();
    initializeHardware();
    if (!CAN_ERROR)
    {
        initializeParticle();

        // Request initial credentials
        requestCredentials();
    }
}

void loop() { handleSystemLoop(); }

void initializeSystem()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    delay(2000);

    // Initialize all subsystems
    initializeConfig();
    initializePorts();
    initializeCredentials();
    initializeMQTT();

    Serial.printlnf("*** KUHMUTE IoT V %s ***", BUILD_VERSION);
    Serial.printlnf("Device ID: %s", Particle.deviceID().c_str());
    Serial.printlnf("Environment: %s", getCurrentEnvironment());
    Serial.printlnf("System initialized");
}

void initializeHardware()
{
    // Setup ring light
    beginLight();
    setLightBlue();

    // Setup SPI for CAN
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV8);

    // Initialize CAN bus
    int err = mcp2515.reset();
    if (err != mcp2515.ERROR_OK)
    {
        reportCANError(err, "reset", true);
        return;
    }

    err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
    if (err != mcp2515.ERROR_OK)
    {
        reportCANError(err, "setBitrate", true);
        return;
    }

    delay(50);
    err = mcp2515.setNormalMode();
    if (err != mcp2515.ERROR_OK)
    {
        reportCANError(err, "setNormalMode", true);
        return;
    }

    pinMode(CAN_INT, INPUT_PULLUP);
    attachInterrupt(CAN_INT, can_interrupt, FALLING);

    new Thread("can_thread", canThread);

    Serial.printlnf("Hardware initialized");
}

void initializeParticle()
{
    // Register cloud functions
    Particle.function("resetDevice", resetDevice);

    // Connect to Particle Cloud
    Particle.connect();
    Particle.publishVitals();
    Particle.keepAlive(PARTICLE_KEEPALIVE_MIN * 60);

    // Enable system features
    System.enableFeature(FEATURE_RESET_INFO);
    System.enableFeature(FEATURE_RETAINED_MEMORY);

    // Wait for connection with timeout
    setLightRed();
    waitFor(Particle.connected, 60000);
    if (!Particle.connected())
    {
        Serial.printlnf("Failed to connect to Particle Cloud");
        resetDevice("Cloud connection timeout");
        return;
    }

    // Log reset reason
    logResetReason();

    setLightBlue();
    Serial.printlnf("Particle Cloud connected");
}

void handleSystemLoop()
{
    // Handle critical errors first
    if (CAN_ERROR)
    {
        blinkCANError();
        return;
    }

    // Check for credential fetch failures
    if (attemptedCredentialsFetchCount > MAX_CREDENTIAL_ATTEMPTS)
    {
        Serial.printlnf("Failed to fetch credentials after max attempts");
        blinkIdentityError();
        return;
    }

    // Handle connection states
    if (!Particle.connected())
    {
        setLightRed();
        return;
    }

    // Handle MQTT and credentials
    handleMQTTClientLoop();
    handleCredentials();
    updateSystemStatus();
}

void canThread()
{
    while (true)
    {
        handleCanQueue();
    }
}

void handleCredentials()
{
    if (areCredentialsValid())
    {
        return; // Already have valid credentials
    }

    if (shouldRetryCredentials())
    {
        requestCredentials();
    }
}

void updateSystemStatus()
{
    if (areCredentialsValid() && isMQTTConnected())
    {
        setLightGreen(); // All systems operational
    }
    else if (areCredentialsValid())
    {
        setLightPurple(); // Have credentials, connecting to MQTT
    }
    else
    {
        setLightBlue(); // Fetching credentials
    }
}
void handleCanQueue()
{
    int messagesProcessed = 0;
    const int MAX_MESSAGES_PER_LOOP =
        8; // Process fewer messages per loop for stability
    int queueMessageCount = 0;

    // Check and handle queue overflow
    if (queueOverflow)
    {
        Serial.println(
            "WARNING: CAN message queue overflow detected - clearing queue");
        noInterrupts();
        queueHead = 0;
        queueTail = 0;
        messageCount = 0;
        queueOverflow = false;
        interrupts();
        // clearAllCANBuffers();
    }

    // Make a safe local copy of the count with interrupts disabled
    noInterrupts();
    queueMessageCount =
        (messageCount > CAN_QUEUE_SIZE) ? CAN_QUEUE_SIZE : messageCount;
    interrupts();

    // Prevent processing if we're approaching a loop timeout
    unsigned long processingStartTime = millis();
    const unsigned long MAX_PROCESSING_TIME =
        3000; // Max 3 seconds for CAN processing

    while (queueMessageCount > 0 &&
           messagesProcessed < MAX_MESSAGES_PER_LOOP)
    {
        // Check if we're taking too long
        if (millis() - processingStartTime > MAX_PROCESSING_TIME)
        {
            Serial.println(
                "WARNING: Breaking CAN processing loop due to timeout");
            break;
        }

        can_frame msg;
        bool validMessage = false;

        noInterrupts(); // Critical section for accessing shared variables
        // Only dequeue if there are still messages
        if (messageCount > 0)
        {
            // Validate queue indices are in valid range before accessing array
            if (queueTail < CAN_QUEUE_SIZE)
            {
                memcpy(&msg, &canMessageQueue[queueTail], sizeof(can_frame));
                queueTail = (queueTail + 1) % CAN_QUEUE_SIZE;
                messageCount--;
                queueMessageCount--;
                validMessage = true;
            }
            else
            {
                // Invalid queue state detected, reset indices
                queueTail = 0;
                queueHead = 0;
                messageCount = 0;
                queueMessageCount = 0;
                Serial.println(
                    "ERROR: Invalid queue indices detected - resetting queue");
            }

            // Track statistics (moved from interrupt handler)
            incrementMessageCounter();
        }
        else
        {
            // Somehow the count changed, break out
            queueMessageCount = 0;
        }
        interrupts();

        // Now safely process the message in main loop context
        if (validMessage)
        {
            receiveMessage(msg);
            messagesProcessed++;
        }

        // Yield more frequently (every 2 messages instead of 3)
        if (messagesProcessed % 2 == 0)
        {
            Particle.process();
        }
    }
}
void receiveMessage(can_frame recMsg)
{
    int addr = recMsg.can_id;
    if (addr <= 0 || addr > MAX_PORTS)
    {
        return;
    }
    Serial.printlnf("MESSAGE FROM ADDRESS: %d", recMsg.can_id);
    Serial.printlnf("%s", recMsg.data);
}

void can_interrupt()
{
    // Minimize work in the interrupt handler
    struct can_frame recMsg;

    // Disable interrupts during critical section
    noInterrupts();

    // Read the message
    int readin = readCanMessage(&recMsg);

    if (readin == ERROR_OK)
    {
        if (messageCount < CAN_QUEUE_SIZE - 1 &&
            queueHead < CAN_QUEUE_SIZE)
        { // Leave one slot as safety margin and
          // validate index
            // Copy the message to the queue without any processing
            memcpy(&canMessageQueue[queueHead], &recMsg, sizeof(can_frame));
            queueHead = (queueHead + 1) % CAN_QUEUE_SIZE;

            // Protect against overflow
            if (messageCount < UINT16_MAX)
            {
                messageCount++;
            }
        }
        else
        {
            // Queue is full or index invalid - set overflow flag and increment
            // counter
            queueOverflow = true;

            // Reset head/tail if they're out of bounds (corruption detection)
            if (queueHead >= CAN_QUEUE_SIZE || queueTail >= CAN_QUEUE_SIZE)
            {
                queueHead = 0;
                queueTail = 0;
                messageCount = 0;
            }
        }
    }
    else if (readin == MCP2515::ERROR_FAIL)
    {
        Serial.printlnf("CAN message error");
    }

    // Clear interrupt flags
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
        char ret[150];
        Serial.printlnf("CAN BUS ERROR %s", operation);
        ReturnErrorString(err, ret, sizeof(ret));
        sprintf(can_err_msg, "CAN BUS ERROR %s: %s", operation, ret);
        Serial.printlnf("%s", can_err_msg);

        // Optionally publish to cloud
        // Particle.publish("CAN ERROR", can_err_msg, PRIVATE);
    }
}

int resetDevice(String command)
{
    // Log reset request with reason if provided
    if (command.length() > 0)
    {
        Serial.printlnf("Device reset requested. Reason: %s", command.c_str());

        // Report last state before reset
        Serial.printlnf("Free memory before reset: %lu bytes", System.freeMemory());
        Serial.printlnf("System uptime: %lu ms", millis());
        Serial.printlnf("MQTT connected: %s", isMQTTConnected() ? "yes" : "no");
        Serial.printlnf("Credentials valid: %s",
                        areCredentialsValid() ? "yes" : "no");

        // Attempt to close connections gracefully
        if (isMQTTConnected())
        {
            Serial.printlnf("Closing MQTT connection...");
        }

        delay(1000); // Give serial time to send
    }

    System.reset();
    return 1; // Won't actually get here
}

void logDebugInfo(const char *checkpoint)
{
    static unsigned long lastLogTime = 0;
    unsigned long now = millis();

    Serial.printlnf("[%lu ms] Checkpoint: %s (Delta: %lu ms)", now, checkpoint,
                    now - lastLogTime);

    lastLogTime = now;
}

void logResetReason()
{
    int reason = System.resetReason();
    int reasonData = System.resetReasonData();

    Serial.printlnf("RESET REASON: %d", reason);
    Serial.printlnf("RESET REASON DATA: %d", reasonData);

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Reason: %d, Data: %d", reason, reasonData);

    Particle.publish("RESET REASON", buffer, PRIVATE);
}

void checkSystemHealth()
{
    // System health monitoring
    unsigned long freeMemory = System.freeMemory();
    unsigned long uptime = millis();

    if (freeMemory < 1000)
    { // Less than 1KB free
        Serial.printlnf("Low memory warning: %lu bytes", freeMemory);
    }

    // Log periodic system status
    static unsigned long lastHealthCheck = 0;
    if (uptime - lastHealthCheck > 60000)
    { // Every minute
        Serial.printlnf("System Health - Uptime: %lu ms, Free Memory: %lu bytes",
                        uptime, freeMemory);
        Serial.printlnf("MQTT Status: %s", getMQTTStatus().c_str());
        Serial.printlnf("Credentials: %s", getCredentialsStatus().c_str());

        lastHealthCheck = uptime;
    }
}
