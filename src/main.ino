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
        Serial.printlnf("CAN message received");
        Serial.printlnf("Data: %s", (char *)recMsg.data);
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
