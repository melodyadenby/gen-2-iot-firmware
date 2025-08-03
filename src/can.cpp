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

uint8_t sendCanMessage(can_frame msg)
{
  uint8_t result = mcp2515.sendMessage(&msg);

  // Basic error tracking - log critical errors
  if (result == 0xA || result == 10 || result == 0xF || result == 15)
  {
    Serial.printlnf("CRITICAL CAN Error: 0x%X - Controller may be corrupted",
                    result);
    // Set global flag for main.ino to handle
    extern volatile bool can_recovery_needed;
    can_recovery_needed = true;
  }
  else if (result != 0)
  {
    Serial.printlnf("CAN Error: 0x%X", result);
  }

  return result;
}

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
uint8_t getCANErrorFlags(bool debugLog)
{
  uint8_t errorFlags = mcp2515.getErrorFlags();
  if (debugLog)
  {
    if (errorFlags == 0)
    {
      return errorFlags; // No errors, exit early
    }

    Serial.println("=== CAN Error Detected ===");

    // --- RX Errors ---
    if (errorFlags & MCP2515::EFLG_RX1OVR)
    {
      Serial.println("RX1OVR: Receive buffer 1 overflow");
    }
    if (errorFlags & MCP2515::EFLG_RX0OVR)
    {
      Serial.println("RX0OVR: Receive buffer 0 overflow");
    }
    if (errorFlags & MCP2515::EFLG_RXEP)
    {
      Serial.println("RXEP: Receive error-passive (high error rate on RX)");
    }

    // --- TX Errors ---
    if (errorFlags & MCP2515::EFLG_TXEP)
    {
      Serial.println("TXEP: Transmit error-passive (too many TX errors)");
    }
    if (errorFlags & MCP2515::EFLG_TXWAR)
    {
      Serial.println("TXWAR: Transmit error warning (high TX error rate)");
    }
    if (errorFlags & MCP2515::EFLG_EWARN)
    {
      Serial.println("EWARN: General error warning (high error count overall)");
    }

    Serial.println("==========================");
  }
  return errorFlags;
}

void printCANErrorState()
{
  uint8_t errorFlags = mcp2515.getErrorFlags();

  if (errorFlags == 0)
  {
    return; // No errors, exit early
  }

  Serial.println("=== CAN Error Detected ===");

  // --- RX Errors ---
  if (errorFlags & MCP2515::EFLG_RX1OVR)
  {
    Serial.println("RX1OVR: Receive buffer 1 overflow");
  }
  if (errorFlags & MCP2515::EFLG_RX0OVR)
  {
    Serial.println("RX0OVR: Receive buffer 0 overflow");
  }
  if (errorFlags & MCP2515::EFLG_RXEP)
  {
    Serial.println("RXEP: Receive error-passive (high error rate on RX)");
  }

  // --- TX Errors ---
  if (errorFlags & MCP2515::EFLG_TXEP)
  {
    Serial.println("TXEP: Transmit error-passive (too many TX errors)");
  }
  if (errorFlags & MCP2515::EFLG_TXWAR)
  {
    Serial.println("TXWAR: Transmit error warning (high TX error rate)");
  }
  if (errorFlags & MCP2515::EFLG_EWARN)
  {
    Serial.println("EWARN: General error warning (high error count overall)");
  }

  Serial.println("==========================");
}
void ReturnErrorString(uint8_t err, char *ret, size_t ret_size)
{
  switch (err)
  {
  case 0:
    snprintf(ret, ret_size, "Error Code: ERROR_OK");
    break;
  case 1:
    snprintf(ret, ret_size, "Error Code: ERROR_FAIL");
    break;
  case 2:
    snprintf(ret, ret_size, "Error Code: ERROR_ALLTXBUSY");
    break;
  case 3:
    snprintf(ret, ret_size, "Error Code: ERROR_FAILINIT");
    break;
  case 4:
    snprintf(ret, ret_size, "Error Code: ERROR_FAILTX");
    break;
  case 5:
    snprintf(ret, ret_size, "Error Code: ERROR_NOMSG");
    break;
  case 10:
    snprintf(ret, ret_size, "Error Code: ERROR_A (0x%X - Controller Error)",
             err);
    break;
  case 15:
    snprintf(ret, ret_size, "Error Code: ERROR_F (0x%X - Bus Error)", err);
    break;
  default:
    if (err >= 10 && err <= 15)
    {
      snprintf(ret, ret_size, "Error Code: ERROR_%X (0x%X - Extended Error)",
               err, err);
    }
    else
    {
      snprintf(ret, ret_size, "Error Code: UNKNOWN (0x%X)", err);
    }
    break;
  }
  Serial.println(ret);
}
