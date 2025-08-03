#include "Particle.h"

#include "DeviceInfoLedger.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);

// If using the last run log, you need to allocate a retained buffer to store the last log
retained uint8_t retainedLogs[2048];

void setup() {
    // The next line is for debugging and waits for USB serial debug to connect for 10 seconds so you can see more early log messages
    waitFor(Serial.isConnected, 10000);

    // It is possible to use entirely local configuration, and only 
    DeviceInfoLedger::instance()
        .withInfoLedgerName("device-info")
        .withLocalConfigLastRunLog(1024)
        .withLocalConfigConnectionLog(2048)
        .withLocalConfigLogAllConnections(false)
        .withLocalConfigIncludeGeneral(true)
        .withLocalConfigIncludeDiag(false)
        .withLocalConfigIncludeTower(false)
        .withLocalConfigLogLevel(LOG_LEVEL_INFO, {})
        .withRetainedBuffer(retainedLogs, sizeof(retainedLogs))
        .setup(); 


    // Using SYSTEM_MODE(SEMI_AUTOMATIC) and calling Particle.connect() after calling setup() for DeviceInfoLedger
    // is recommended to avoid getting new connection information in the last run log.
    Particle.connect();

}

void loop() {
    DeviceInfoLedger::instance().loop();
}