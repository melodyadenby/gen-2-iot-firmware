#include "Arduino.h"
#include "Particle.h"
#include <stdarg.h>
bool checkMsgIsTrueFalse(unsigned char data[8], int length)
{
  return std::all_of(data, data + length,
                     [](unsigned char c)
                     { return c == 'T' || c == 'F'; });
}

bool isVINFull(const char VIN[16]) { return strnlen(VIN, 16) == 16; }

bool isVINEmpty(const char VIN[16])
{
  return VIN[0] == '\0' || strcmp(VIN, "-") != 0;
}

// Safe string copy function to prevent buffer overflows
void safeStrCopy(char *dest, const char *src, size_t destSize)
{
  if (dest == NULL || src == NULL || destSize == 0)
  {
    return;
  }

  size_t i;
  for (i = 0; i < destSize - 1 && src[i] != '\0'; i++)
  {
    dest[i] = src[i];
  }
  dest[i] = '\0'; // Always ensure null termination
}
