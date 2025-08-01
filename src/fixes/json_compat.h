#ifndef JSON_COMPAT_H
#define JSON_COMPAT_H

// This header fixes compatibility issues between ArduinoJson library and
// Particle firmware The main issue is that Particle's pgmspace.h defines
// pgm_read_ptr in a way that's incompatible with ArduinoJson's templated
// pgm_read function expectations.
//
// IMPORTANT: This file must be included BEFORE any ArduinoJson includes!

// Disable PROGMEM support entirely in ArduinoJson to avoid conflicts
// This is the most reliable way to prevent pgmspace-related compilation errors
#define ARDUINOJSON_ENABLE_PROGMEM 0

// Also disable other PROGMEM-related features that might cause issues
#define ARDUINOJSON_USE_LONG_LONG 0
#define ARDUINOJSON_USE_DOUBLE 1

// If we still get conflicts, undefine the problematic macros
#ifdef pgm_read_ptr
#undef pgm_read_ptr
#endif

#ifdef pgm_read_byte
#undef pgm_read_byte
#endif

#ifdef pgm_read_word
#undef pgm_read_word
#endif

#ifdef pgm_read_dword
#undef pgm_read_dword
#endif

#ifdef pgm_read_float
#undef pgm_read_float
#endif

#ifdef pgm_read_double
#undef pgm_read_double
#endif

#endif // JSON_COMPAT_H
