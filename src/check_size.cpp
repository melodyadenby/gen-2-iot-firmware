#include "Arduino.h"
#include "diagnostics.h"
#include <stdio.h>

void checkDiagnosticsSize() {
    Serial.println("=== EEPROM USAGE ANALYSIS ===");
    
    // Calculate size of diagnostic structure
    size_t structSize = sizeof(DiagnosticData);
    
    Serial.printlnf("DiagnosticData structure size: %d bytes", structSize);
    Serial.printlnf("EEPROM total size: 4096 bytes");
    Serial.printlnf("EEPROM used: %.1f%%", (structSize * 100.0) / 4096.0);
    Serial.printlnf("EEPROM remaining: %d bytes", 4096 - structSize);
    
    // Break down the structure size
    Serial.println("\n--- Structure Breakdown ---");
    Serial.printlnf("Magic + Version + Checksum: %d bytes", 
                    sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint32_t));
    Serial.printlnf("Timestamps: %d bytes", 
                    sizeof(uint32_t) + sizeof(int32_t));
    Serial.printlnf("CAN health: ~%d bytes", 
                    sizeof(uint32_t) + 6 * sizeof(uint8_t) + sizeof(bool));
    Serial.printlnf("System health: %d bytes", 
                    3 * sizeof(uint32_t));
    Serial.printlnf("Network health: ~%d bytes", 
                    2 * sizeof(uint32_t) + 2 * sizeof(bool) + sizeof(uint8_t));
    Serial.printlnf("Interrupt health: ~%d bytes", 
                    2 * sizeof(uint32_t) + sizeof(bool));
    Serial.printlnf("Port timestamps [%d ports]: %d bytes", 
                    MAX_PORTS, MAX_PORTS * sizeof(int32_t));
    Serial.printlnf("Port statistics: ~%d bytes", 
                    sizeof(uint8_t) * 2 + sizeof(uint32_t));
    Serial.printlnf("Message statistics: %d bytes", 
                    3 * sizeof(uint32_t));
    Serial.printlnf("Reset info: ~%d bytes", 
                    2 * sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint16_t));
    Serial.printlnf("Error strings: %d bytes", 
                    48 + 16);  // lastError[48] + lastCANOperation[16]
    Serial.printlnf("Environmental: ~%d bytes", 
                    sizeof(int8_t) + sizeof(uint8_t) + sizeof(int16_t));
    Serial.printlnf("Reserved space: %d bytes", 32);
    
    Serial.println("\n--- EEPROM Layout ---");
    Serial.printlnf("Address 0x0000 - 0x%04X: DiagnosticData (%d bytes)", 
                    structSize - 1, structSize);
    Serial.printlnf("Address 0x%04X - 0x0FFF: Free space (%d bytes)", 
                    structSize, 4096 - structSize);
    
    // Multiple diagnostic sets possibility
    Serial.println("\n--- Multiple Storage Options ---");
    int maxSets = 4096 / structSize;
    Serial.printlnf("Could store %d complete diagnostic sets", maxSets);
    Serial.printlnf("Could implement rotating logs with %d historical snapshots", maxSets - 1);
    
    // Wear leveling estimation
    Serial.println("\n--- EEPROM Wear Leveling ---");
    Serial.println("Particle EEPROM has ~100,000 write cycles per cell");
    Serial.printlnf("With 1-minute save interval: ~69 days continuous operation");
    Serial.printlnf("With 5-minute save interval: ~347 days continuous operation");
    Serial.printlnf("With 10-minute save interval: ~694 days continuous operation");
    
    // Recommendations
    Serial.println("\n--- Recommendations ---");
    if (structSize > 1024) {
        Serial.println("⚠️  Structure uses >25% of EEPROM");
        Serial.println("   Consider reducing reserved space or string sizes");
    } else {
        Serial.println("✓  Structure size is reasonable");
    }
    
    if (structSize <= 512) {
        Serial.println("✓  Could implement dual-bank redundancy");
        Serial.println("   Store two copies for extra reliability");
    }
    
    Serial.println("\n=== END ANALYSIS ===");
}

// Function to display actual memory addresses
void displayEEPROMMap() {
    Serial.println("\n=== EEPROM MEMORY MAP ===");
    
    size_t diagnosticsSize = sizeof(DiagnosticData);
    
    // Show what's currently allocated
    Serial.println("Current Allocation:");
    Serial.printlnf("  0x0000-0x%04X: Primary Diagnostics (%d bytes)", 
                    diagnosticsSize-1, diagnosticsSize);
    
    // Suggest additional uses for remaining space
    size_t remaining = 4096 - diagnosticsSize;
    size_t offset = diagnosticsSize;
    
    Serial.println("\nPotential Additional Uses:");
    
    if (remaining >= diagnosticsSize) {
        Serial.printlnf("  0x%04X-0x%04X: Backup Diagnostics (%d bytes)", 
                        offset, offset + diagnosticsSize - 1, diagnosticsSize);
        offset += diagnosticsSize;
        remaining -= diagnosticsSize;
    }
    
    if (remaining >= 256) {
        Serial.printlnf("  0x%04X-0x%04X: Configuration Data (256 bytes)", 
                        offset, offset + 255);
        offset += 256;
        remaining -= 256;
    }
    
    if (remaining >= 128) {
        Serial.printlnf("  0x%04X-0x%04X: Event Log Buffer (128 bytes)", 
                        offset, offset + 127);
        offset += 128;
        remaining -= 128;
    }
    
    if (remaining > 0) {
        Serial.printlnf("  0x%04X-0x0FFF: Unused (%d bytes)", 
                        offset, remaining);
    }
    
    Serial.println("\n=== END MEMORY MAP ===");
}