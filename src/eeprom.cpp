#include "eeprom.h"

// Static member definitions
bool EEPROMManager::initialized = false;
unsigned long EEPROMManager::lastSaveTime = 0;
bool EEPROMManager::needsSave = false;

// Global instance
EEPROMManager eepromManager;

void EEPROMManager::init() {
    Log.info("Initializing EEPROM Manager...");
    
    // Check if EEPROM has valid data
    if (!verifyEEPROM()) {
        Log.info("EEPROM invalid or empty, initializing with defaults...");
        clearEEPROM();
        writeHeader();
        saveAllPortMessageTimes();
    } else {
        Log.info("EEPROM valid, loading port message times...");
        loadAllPortMessageTimes();
    }
    
    lastSaveTime = millis();
    initialized = true;
    needsSave = false;
    
    Log.info("EEPROM Manager initialized successfully");
}

bool EEPROMManager::isInitialized() {
    return initialized;
}

void EEPROMManager::savePortMessageTime(int portNumber) {
    if (!isValidPort(portNumber)) {
        Log.error("Invalid port number: %d", portNumber);
        return;
    }
    
    PortState *state = getPortState(portNumber);
    if (state == nullptr) {
        Log.error("Failed to get port state for port %d", portNumber);
        return;
    }
    
    // Calculate address for this port's timestamp
    int addr = EEPROM_PORT_TIMES_ADDR + ((portNumber - 1) * sizeof(unsigned long));
    
    // Write the timestamp
    EEPROM.put(addr, state->last_port_message);
    
    // Mark that we need to update checksum
    needsSave = true;
    
    Log.trace("Saved port %d message time: %lu", portNumber, state->last_port_message);
}

void EEPROMManager::saveAllPortMessageTimes() {
    Log.info("Saving all port message times to EEPROM...");
    
    for (int i = 1; i <= MAX_PORTS; i++) {
        PortState *state = getPortState(i);
        if (state != nullptr) {
            int addr = EEPROM_PORT_TIMES_ADDR + ((i - 1) * sizeof(unsigned long));
            EEPROM.put(addr, state->last_port_message);
        }
    }
    
    // Update checksum
    uint32_t checksum = calculateChecksum();
    EEPROM.put(EEPROM_CHECKSUM_ADDR, checksum);
    
    lastSaveTime = millis();
    needsSave = false;
    
    Log.info("All port message times saved successfully");
}

void EEPROMManager::loadAllPortMessageTimes() {
    Log.info("Loading port message times from EEPROM...");
    
    for (int i = 1; i <= MAX_PORTS; i++) {
        PortState *state = getPortState(i);
        if (state != nullptr) {
            int addr = EEPROM_PORT_TIMES_ADDR + ((i - 1) * sizeof(unsigned long));
            unsigned long timestamp;
            EEPROM.get(addr, timestamp);
            
            // Sanity check - don't load timestamps from the future
            if (timestamp <= millis() || timestamp == 0) {
                state->last_port_message = timestamp;
                Log.trace("Loaded port %d message time: %lu", i, timestamp);
            } else {
                Log.warn("Invalid timestamp for port %d: %lu (future time), resetting to 0", i, timestamp);
                state->last_port_message = 0;
            }
        }
    }
    
    Log.info("Port message times loaded successfully");
}

unsigned long EEPROMManager::getStoredPortMessageTime(int portNumber) {
    if (!isValidPort(portNumber)) {
        Log.error("Invalid port number: %d", portNumber);
        return 0;
    }
    
    int addr = EEPROM_PORT_TIMES_ADDR + ((portNumber - 1) * sizeof(unsigned long));
    unsigned long timestamp;
    EEPROM.get(addr, timestamp);
    
    return timestamp;
}

void EEPROMManager::checkAndSave() {
    if (!initialized) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // Check if we need to save (either marked as needs save or periodic save interval reached)
    if (needsSave || (currentTime - lastSaveTime >= EEPROM_SAVE_INTERVAL)) {
        // Ensure minimum interval between saves
        if (currentTime - lastSaveTime >= EEPROM_MIN_SAVE_INTERVAL) {
            saveAllPortMessageTimes();
            Log.info("Auto-saved port message times to EEPROM");
        }
    }
}

void EEPROMManager::forceSave() {
    if (!initialized) {
        Log.warn("Cannot force save - EEPROM Manager not initialized");
        return;
    }
    
    saveAllPortMessageTimes();
    Log.info("Force saved port message times to EEPROM");
}

void EEPROMManager::clearEEPROM() {
    Log.info("Clearing EEPROM...");
    
    // Clear all EEPROM data in our range
    for (int i = 0; i < EEPROM_TOTAL_SIZE; i++) {
        EEPROM.write(i, 0);
    }
    
    Log.info("EEPROM cleared");
}

void EEPROMManager::printEEPROMStatus() {
    Log.info("=== EEPROM Status ===");
    Log.info("Initialized: %s", initialized ? "Yes" : "No");
    Log.info("Valid: %s", verifyEEPROM() ? "Yes" : "No");
    Log.info("Last save: %lu ms ago", millis() - lastSaveTime);
    Log.info("Needs save: %s", needsSave ? "Yes" : "No");
    
    // Print stored port times
    Log.info("Port Message Times:");
    for (int i = 1; i <= MAX_PORTS; i++) {
        unsigned long storedTime = getStoredPortMessageTime(i);
        PortState *state = getPortState(i);
        unsigned long currentTime = state ? state->last_port_message : 0;
        
        if (storedTime > 0 || currentTime > 0) {
            Log.info("  Port %d: Stored=%lu, Current=%lu, Age=%lu ms", 
                     i, storedTime, currentTime, 
                     currentTime > 0 ? (millis() - currentTime) : 0);
        }
    }
    Log.info("===================");
}

bool EEPROMManager::verifyEEPROM() {
    // Check magic number
    uint32_t magic;
    EEPROM.get(EEPROM_MAGIC_ADDR, magic);
    if (magic != EEPROM_MAGIC_NUMBER) {
        Log.trace("EEPROM magic number mismatch: 0x%08X != 0x%08X", magic, EEPROM_MAGIC_NUMBER);
        return false;
    }
    
    // Check version
    uint32_t version;
    EEPROM.get(EEPROM_VERSION_ADDR, version);
    if (version != EEPROM_VERSION) {
        Log.trace("EEPROM version mismatch: %d != %d", version, EEPROM_VERSION);
        return false;
    }
    
    // Verify checksum
    return verifyChecksum();
}

uint32_t EEPROMManager::calculateChecksum() {
    uint32_t checksum = 0;
    
    // Calculate checksum over all port timestamps
    for (int i = 0; i < MAX_PORTS; i++) {
        int addr = EEPROM_PORT_TIMES_ADDR + (i * sizeof(unsigned long));
        unsigned long timestamp;
        EEPROM.get(addr, timestamp);
        
        // Simple checksum - XOR all bytes
        for (int j = 0; j < sizeof(unsigned long); j++) {
            checksum ^= ((timestamp >> (j * 8)) & 0xFF);
            checksum = (checksum << 1) | (checksum >> 31); // Rotate left by 1
        }
    }
    
    return checksum;
}

bool EEPROMManager::verifyChecksum() {
    uint32_t storedChecksum;
    EEPROM.get(EEPROM_CHECKSUM_ADDR, storedChecksum);
    
    uint32_t calculatedChecksum = calculateChecksum();
    
    bool valid = (storedChecksum == calculatedChecksum);
    if (!valid) {
        Log.trace("EEPROM checksum mismatch: stored=0x%08X, calculated=0x%08X", 
                  storedChecksum, calculatedChecksum);
    }
    
    return valid;
}

void EEPROMManager::writeHeader() {
    Log.trace("Writing EEPROM header...");
    
    // Write magic number
    EEPROM.put(EEPROM_MAGIC_ADDR, (uint32_t)EEPROM_MAGIC_NUMBER);
    
    // Write version
    EEPROM.put(EEPROM_VERSION_ADDR, (uint32_t)EEPROM_VERSION);
    
    Log.trace("EEPROM header written");
}

bool EEPROMManager::readHeader() {
    uint32_t magic, version;
    
    EEPROM.get(EEPROM_MAGIC_ADDR, magic);
    EEPROM.get(EEPROM_VERSION_ADDR, version);
    
    return (magic == EEPROM_MAGIC_NUMBER && version == EEPROM_VERSION);
}