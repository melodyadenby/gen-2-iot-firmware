#include "Arduino.h"
#include "Particle.h"
#include "can.h"
#include "can_processor.h"
#include "config.h"
#include "credentials.h"
#include "fixes/json_compat.h"
#include "lights.h"
#include "main.h"
#include "mqtt.h"
#include "port_event_handler.h"
#include "port_flag_handler.h"
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

// Architecture components
PortEventHandler *portEventHandler = nullptr;
PortFlagHandler *portFlagHandler = nullptr;

volatile bool queueOverflow = false;
volatile int messageCount = 0;
volatile int queueHead = 0;
volatile int queueTail = 0;
can_frame messageQueue[50];

void setup()
{
    initializeSystem();
    initializeArchitecture();
    initializeHardware();

    if (!CAN_ERROR)
    {
        initializeParticle();
        requestCredentials();
    }
}

void loop()
{
    handleSystemLoop();
}

void initializeArchitecture()
{
    // Initialize the clean architecture components
    portEventHandler = new PortEventHandler(nullptr); // Will use global port state functions
    portFlagHandler = new PortFlagHandler(nullptr);   // Will use global port state functions

    Serial.printlnf("Architecture components initialized");
}

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

    // Start CAN processing thread
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

    // System health monitoring
    checkSystemHealth();
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

// ========================================
// CAN Processing
// ========================================

void canThread()
{
    Serial.printlnf("CAN processing thread started");

    while (true)
    {
        // Process incoming CAN messages
        handleCanQueue();

        // Process port flags (replaces old flagHandler)
        if (portFlagHandler)
        {
            portFlagHandler->processAllPortFlags();
        }

        // Small delay to prevent busy-waiting
        delay(10);
    }
}

void handleCanQueue()
{
    int messagesProcessed = 0;
    const int MAX_MESSAGES_PER_LOOP = 8;

    while (messageCount > 0 && messagesProcessed < MAX_MESSAGES_PER_LOOP)
    {
        can_frame msg;
        bool validMessage = false;

        // Thread-safe message extraction
        noInterrupts();
        if (messageCount > 0)
        {
            if (queueHead >= 0 && queueHead < 50 && queueTail >= 0 && queueTail < 50)
            {
                msg = messageQueue[queueHead];
                queueHead = (queueHead + 1) % 50;
                messageCount--;
                validMessage = true;

                // Handle queue overflow reset
                if (queueOverflow && messageCount < 25)
                {
                    queueOverflow = false;
                    Serial.println("Queue overflow cleared");
                }
            }
            else
            {
                // Invalid queue state, reset
                queueTail = 0;
                queueHead = 0;
                messageCount = 0;
                Serial.println("ERROR: Invalid queue indices detected - resetting queue");
            }
        }
        interrupts();

        // Process the message using clean architecture
        if (validMessage)
        {
            processCANMessage(msg);
            messagesProcessed++;
        }

        // Yield periodically
        if (messagesProcessed % 2 == 0)
        {
            Particle.process();
        }
    }
}

void processCANMessage(const can_frame &rawMessage)
{
    // 1. Parse the raw CAN message
    ParsedCANMessage parsedMsg = canProcessor.parseMessage(rawMessage);

    // 2. Log the message for debugging
    Serial.printlnf("CAN message from port %d: type=%s, valid=%s",
                    parsedMsg.sourcePort,
                    canProcessor.getMessageTypeString(parsedMsg.messageType),
                    parsedMsg.isValid ? "yes" : "no");

    // 3. Handle the business logic if message is valid
    if (parsedMsg.isValid && portEventHandler)
    {
        portEventHandler->handleCANMessage(parsedMsg);
    }
    else if (!parsedMsg.isValid)
    {
        Serial.printlnf("Invalid CAN message received from port %d", parsedMsg.sourcePort);
    }
}

void can_interrupt()
{
    // Minimal interrupt handler - just queue the message
    struct can_frame recMsg;

    // Read the message
    int readin = readCanMessage(&recMsg);

    if (readin == ERROR_OK)
    {
        // Add to queue if there's space
        if (messageCount < 50)
        {
            messageQueue[queueTail] = recMsg;
            queueTail = (queueTail + 1) % 50;
            messageCount++;
        }
        else
        {
            queueOverflow = true;
        }
    }

    // Clear interrupt flags
    clearCanInterrupts();
}

// ========================================
// Utility Functions (Unchanged)
// ========================================

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
    }
}

int resetDevice(String command)
{
    if (command.length() > 0)
    {
        Serial.printlnf("Device reset requested. Reason: %s", command.c_str());
        Serial.printlnf("Free memory before reset: %lu bytes", System.freeMemory());
        Serial.printlnf("System uptime: %lu ms", millis());
        Serial.printlnf("MQTT connected: %s", isMQTTConnected() ? "yes" : "no");
        Serial.printlnf("Credentials valid: %s", areCredentialsValid() ? "yes" : "no");

        if (isMQTTConnected())
        {
            Serial.printlnf("Closing MQTT connection...");
        }

        delay(1000);
    }

    System.reset();
    return 1;
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
    unsigned long freeMemory = System.freeMemory();
    unsigned long uptime = millis();

    if (freeMemory < 1000)
    {
        Serial.printlnf("Low memory warning: %lu bytes", freeMemory);
    }

    static unsigned long lastHealthCheck = 0;
    if (uptime - lastHealthCheck > 60000)
    {
        Serial.printlnf("System Health - Uptime: %lu ms, Free Memory: %lu bytes",
                        uptime, freeMemory);
        Serial.printlnf("MQTT Status: %s", getMQTTStatus().c_str());
        Serial.printlnf("Credentials: %s", getCredentialsStatus().c_str());

        if (portFlagHandler)
        {
            Serial.printlnf("Ports with pending flags: %d", portFlagHandler->getPendingPortsCount());
        }

        lastHealthCheck = uptime;
    }
}
