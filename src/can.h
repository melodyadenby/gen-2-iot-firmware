#ifndef CAN_H
#define CAN_H

#include "mcp2515.h" // Make sure the MCP2515 class is defined

#define CAN_CS A5
#define CAN_INT A4

// Forward declaration of interrupt handler
void can_interrupt();
extern MCP2515
    mcp2515; // Declare the mcp2515 object here, without instantiating

extern uint32_t total_messages_received;
extern uint32_t last_message_time;
extern uint32_t rapid_message_count;

const int ERROR_OK = 0;

MCP2515::ERROR readCanMessage(struct can_frame *msg);
uint8_t sendCanMessage(can_frame msg);
void clearCanInterrupts();
void ReturnErrorString(uint8_t err, char *ret, size_t ret_size);

uint8_t getCANErrorFlags(bool debugLog);
void printCANErrorState();
void clearAllCANBuffers();
void emergencyMCP2515Reset();
// int initCAN();
void incrementMessageCounter();

#endif // CAN_H
