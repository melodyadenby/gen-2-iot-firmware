#ifndef UTILS_H
#define UTILS_H

#include <stddef.h> // For size_t

bool checkMsgIsTrueFalse(unsigned char data[8], int length);
bool isVINFull(const char VIN[16]);
bool isVINEmpty(const char VIN[16]);

// Safe string copy function to prevent buffer overflows
void safeStrCopy(char *dest, const char *src, size_t destSize);

#endif // UTILS_H
