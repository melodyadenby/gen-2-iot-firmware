#ifndef SERIAL_FIX_H
#define SERIAL_FIX_H

#include "Arduino.h"
#include "Particle.h"
#include <stdarg.h>

// Custom printlnf that automatically adds newline after every call
// This guarantees clean output and prevents fragmentation
inline void SerialPrintfAuto(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  Serial.printlnf(format, args);
  va_end(args);
  Serial.println(); // Automatically add newline - this flushes and prevents
                    // fragmentation
}

// Use this instead of Serial.printlnf() to get clean, unfragmented output
#define printlnf(...) SerialPrintfAuto(__VA_ARGS__)

// For cases where you need the original Serial.printlnf behavior (rare)
#define PrintfNoNewline(...) Serial.printlnf(__VA_ARGS__)

#endif // SERIAL_FIX_H
