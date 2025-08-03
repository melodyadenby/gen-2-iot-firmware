#include "Particle.h"

#include "DeviceInfoLedger.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);

retained uint8_t retainedLogs[2048];

int testHandler(String cmd);

bool testSleep = false;

void setup() {
    // The next line is for debugging and waits for USB serial debug to connect for 10 seconds so you can see more early log messages
    waitFor(Serial.isConnected, 10000);

    Particle.function("test", testHandler);

    // This sets up remote configuration
    DeviceConfigLedger::instance()
        .withConfigDefaultLedgerName("device-info-defaults")
        .setup();

    // This sets up the device information in ledger
    DeviceInfoLedger::instance()
        .withInfoLedgerName("device-info")
        .withRetainedBuffer(retainedLogs, sizeof(retainedLogs))
        .setup(); 


    // Using SYSTEM_MODE(SEMI_AUTOMATIC) and calling Particle.connect() after calling setup() for DeviceInfoLedger
    // is recommended to avoid getting new connection information in the last run log.
    Particle.connect();

}

void loop() {
    DeviceInfoLedger::instance().loop();

    if (testSleep) {
        testSleep = false;
        delay(2000);
        
        SystemSleepConfiguration config;
        config.mode(SystemSleepMode::STOP)
            .duration(30s);
        System.sleep(config);
    }
}

int testHandler(String cmd) {
    int cmdInt = cmd.toInt();

    switch(cmdInt) {
        case 1:
            testSleep = true;
            break;
    }

    return 0;
}
