#include "can.h"
#include "Arduino.h"
#include "Particle.h"

// Create MCP2515 instance with CS pin
MCP2515 mcp2515(CAN_CS);

uint32_t total_messages_received = 0;
uint32_t last_message_time = 0;
uint32_t rapid_message_count = 0;

MCP2515::ERROR readCanMessage(struct can_frame *msg)
{
    // Use the provided msg pointer to store the message
    return mcp2515.readMessage(msg);
}

uint8_t sendCanMessage(can_frame msg) { return mcp2515.sendMessage(&msg); }

void clearCanInterrupts() { mcp2515.clearInterrupts(); }
void incrementMessageCounter()
{
    total_messages_received++;
    rapid_message_count++;

    // Monitor for back-to-back messages (potential flood)
    unsigned long current_time = millis();
    if (last_message_time > 0 && (current_time - last_message_time) < 10)
    {
        // Back-to-back messages detected (less than 10ms apart)
        if (rapid_message_count > 10)
        {
            Serial.printlnf("ALERT: Rapid messages detected - %lums apart",
                            current_time - last_message_time);
        }
    }
    last_message_time = current_time;
}

void ReturnErrorString(uint8_t err, char *ret, size_t ret_size)
{
    switch (err)
    {
    case 0:
        snprintf(ret, ret_size, "Error Code: ERROR_OK, %d", err);
        break;
    case 1:
        snprintf(ret, ret_size, "Error Code: ERROR_FAIL, %d", err);
        break;
    case 2:
        snprintf(ret, ret_size, "Error Code: ERROR_ALLTXBUSY, %d", err);
        break;
    case 3:
        snprintf(ret, ret_size, "Error Code: ERROR_FAILINIT, %d", err);
        break;
    case 4:
        snprintf(ret, ret_size, "Error Code: ERROR_FAILTX, %d", err);
        break;
    case 5:
        snprintf(ret, ret_size, "Error Code: ERROR_NOMSG, %d", err);
        break;
    default:
        snprintf(ret, ret_size, "Error Code: UNKNOWN, %d", err);
        break;
    }
    Serial.println(ret);
}
