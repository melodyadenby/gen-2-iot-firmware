#ifndef EEPROM_H
#define EEPROM_H

#include "Arduino.h"
#include "Particle.h"
#include "config.h"
#include "port_state.h"

// EEPROM memory layout constants
const int EEPROM_MAGIC_NUMBER = 0xDEADBEEF;  // Magic number to verify EEPROM validity
const int EEPROM_VERSION = 1;                 // EEPROM data structure version

// EEPROM addresses
const int EEPROM_MAGIC_ADDR = 0;             // 4 bytes
const int EEPROM_VERSION_ADDR = 4;           // 4 bytes  
const int EEPROM_PORT_TIMES_ADDR = 8;        // Start of port message times array
const int EEPROM_CHECKSUM_ADDR = 8 + (MAX_PORTS * sizeof(unsigned long)); // Checksum at end

// EEPROM size calculation
const int EEPROM_TOTAL_SIZE = EEPROM_CHECKSUM_ADDR + sizeof(uint32_t);

// Auto-save configuration
const unsigned long EEPROM_SAVE_INTERVAL = 5 * 60 * 1000;  // Save every 5 minutes
const unsigned long EEPROM_MIN_SAVE_INTERVAL = 10 * 1000;  // Minimum 10 seconds between saves

// EEPROM management class
class EEPROMManager {
public:
    // Initialization
    static void init();
    static bool isInitialized();
    
    // Port message time management
    static void savePortMessageTime(int portNumber);
    static void saveAllPortMessageTimes();
    static void loadAllPortMessageTimes();
    static unsigned long getStoredPortMessageTime(int portNumber);
    
    // Periodic save management
    static void checkAndSave();
    static void forceSave();
    
    // Debug and utility functions
    static void clearEEPROM();
    static void printEEPROMStatus();
    static bool verifyEEPROM();
    
private:
    static bool initialized;
    static unsigned long lastSaveTime;
    static bool needsSave;
    
    // Helper functions
    static uint32_t calculateChecksum();
    static bool verifyChecksum();
    static void writeHeader();
    static bool readHeader();
};

// Global instance
extern EEPROMManager eepromManager;

#endif // EEPROM_H