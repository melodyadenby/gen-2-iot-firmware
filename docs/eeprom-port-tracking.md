# EEPROM Port Message Tracking

## Overview

The EEPROM Port Message Tracking feature provides persistent storage of the last message timestamps received from each CAN port. This allows the IoT device to maintain port communication history across reboots and power cycles.

## Features

- **Persistent Storage**: Port message timestamps are saved to EEPROM and survive device restarts
- **Automatic Updates**: Every CAN message received automatically updates the timestamp
- **Periodic Auto-Save**: Timestamps are automatically saved to EEPROM every 5 minutes
- **Data Integrity**: Checksum validation ensures data integrity
- **Cloud Functions**: Remote monitoring and management via Particle cloud functions

## How It Works

### Message Reception
When a CAN message is received:
1. The `CANMessageProcessor::parseMessage()` function captures the message
2. The port's `last_port_message` timestamp is updated with `millis()`
3. The EEPROM manager is notified to mark the data for saving

### EEPROM Structure
The EEPROM layout is organized as follows:
```
Address 0-3:    Magic Number (0xDEADBEEF)
Address 4-7:    Version Number (1)
Address 8-135:  Port Message Times Array (16 ports × 8 bytes each)
Address 136-139: Checksum
```

### Auto-Save Mechanism
- Data is automatically saved every 5 minutes (`EEPROM_SAVE_INTERVAL`)
- Minimum 10 seconds between saves to prevent excessive writes
- Manual saves can be triggered via cloud functions

## Implementation Files

### Core Files
- `eeprom.h` - EEPROM manager class definition
- `eeprom.cpp` - EEPROM manager implementation
- `can_processor.cpp` - Modified to update timestamps on message reception
- `main.cpp` - Integration with system initialization and main loop

### Key Functions

#### Initialization
```cpp
eepromManager.init();
```
Called during system startup to:
- Verify EEPROM data integrity
- Load saved port message times
- Initialize fresh EEPROM if needed

#### Automatic Updates
```cpp
// In can_processor.cpp
PortState *state = getPortState(rawMessage.can_id);
if (state != nullptr) {
    state->last_port_message = millis();
    eepromManager.savePortMessageTime(rawMessage.can_id);
}
```

#### Periodic Saves
```cpp
// In main loop
eepromManager.checkAndSave();
```

## Cloud Functions

### eepromStatus
Prints detailed EEPROM status including:
- Initialization state
- Data validity
- Last save time
- All port message timestamps

**Usage**: `particle call <device> eepromStatus`

### eepromSave
Forces immediate save of all port message times to EEPROM.

**Usage**: `particle call <device> eepromSave`

### eepromClear
Clears all EEPROM data and reinitializes. Requires "CONFIRM" parameter for safety.

**Usage**: `particle call <device> eepromClear CONFIRM`

### portMsgTimes
Returns JSON object with current port message timestamps.

**Usage**: `particle call <device> portMsgTimes`

**Example Response**:
```json
{"1":1234567890,"3":1234567895,"7":1234567900}
```

## Usage Examples

### Monitoring Port Communication
```bash
# Check which ports have communicated recently
particle call my-device portMsgTimes

# Get detailed EEPROM status
particle call my-device eepromStatus
```

### Debugging Communication Issues
1. Check if a specific port has ever communicated:
   - Look at the `portMsgTimes` output
   - Missing ports have never sent a message
   
2. Calculate time since last message:
   - Current time (millis) - port timestamp = milliseconds since last message

### Manual Maintenance
```bash
# Force save current data
particle call my-device eepromSave

# Clear all stored data (use with caution)
particle call my-device eepromClear CONFIRM
```

## Technical Details

### Memory Usage
- Total EEPROM usage: 140 bytes
- Per-port storage: 8 bytes (unsigned long timestamp)
- Supports up to 16 ports (MAX_PORTS)

### Data Persistence
- Magic number validates EEPROM initialization
- Version number allows future format changes
- Checksum ensures data integrity
- Automatic recovery from corrupted data

### Performance Considerations
- EEPROM writes are minimized with intelligent caching
- Updates are marked but not immediately written
- Batch writes occur periodically or on demand
- System reset triggers automatic save

### Error Handling
- Invalid port numbers are rejected
- Future timestamps are reset to 0
- Corrupted EEPROM data triggers reinitialization
- All operations are logged for debugging

## Integration Points

### System Startup
The EEPROM manager is initialized in `initializeSystem()` after ports are initialized:
```cpp
initializePorts();
initializeCredentials();
initializeLedger();
eepromManager.init();  // Restore port message times
```

### System Shutdown
A reset handler ensures data is saved before shutdown:
```cpp
System.on(reset, resetHandler);
```

### Main Loop
Periodic saves are checked in the main loop:
```cpp
eepromManager.checkAndSave();
```

## Best Practices

1. **Regular Monitoring**: Use `eepromStatus` periodically to verify data integrity
2. **Debug New Installations**: Check `portMsgTimes` to confirm all ports are communicating
3. **Avoid Excessive Saves**: Let the auto-save handle periodic updates
4. **Clear Carefully**: Only use `eepromClear` when absolutely necessary
5. **Check Logs**: EEPROM operations are logged at various levels for debugging

## Troubleshooting

### No Data Saved
- Verify EEPROM initialization in logs
- Check for "EEPROM Manager initialized successfully" message
- Use `eepromStatus` to check validity

### Incorrect Timestamps
- Timestamps in the future are automatically reset
- Check system time is correct
- Verify port is actually sending messages

### Data Loss After Reboot
- Check EEPROM checksum validation in logs
- Verify magic number and version match
- Look for EEPROM corruption messages

## Future Enhancements

Potential improvements for future versions:
- Message count tracking per port
- Last message type received
- Error count tracking
- Configurable save intervals
- Extended statistics storage