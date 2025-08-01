#ifndef CAN_H
#define CAN_H

#include "mcp2515.h" // Make sure the MCP2515 class is defined

#define CAN_CS A5
extern MCP2515 mcp2515; // Declare the mcp2515 object here, without instantiating

const int ERROR_OK = 0;

MCP2515::ERROR readCanMessage(struct can_frame *msg);
uint8_t sendCanMessage(can_frame msg);
void clearCanInterrupts();
void ReturnErrorString(uint8_t err, char *ret, size_t ret_size);
// int initCAN();

#endif // CAN_H