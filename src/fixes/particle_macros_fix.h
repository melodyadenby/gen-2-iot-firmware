#ifndef PARTICLE_MACROS_FIX_H
#define PARTICLE_MACROS_FIX_H

// This header provides definitions to fix Particle product macros
// for clangd and other IDEs that have issues with Particle's macro system

#ifndef PRODUCT_ID
// Fallback product ID for IDE compatibility
#define PRODUCT_ID 12345
#endif

#ifndef PRODUCT_VERSION
// Fallback product version for IDE compatibility
#define PRODUCT_VERSION 34
#endif

#ifndef PRODUCT_VERSION_STRING
// Define PRODUCT_VERSION_STRING which Particle often uses
#define PRODUCT_VERSION_STRING "2.0"
#endif

// Application product ID wrapper (needed for proper parsing)
#ifndef __ApplicationProductID
class __ApplicationProductID {
public:
    __ApplicationProductID(int id) {}
};
#endif

// Application product version wrapper (needed for proper parsing)
#ifndef __ApplicationProductVersion
class __ApplicationProductVersion {
public:
    __ApplicationProductVersion(int version) {}
};
#endif

// System thread wrapper for IDE compatibility
#ifndef SYSTEM_THREAD
#define SYSTEM_THREAD(x) 
#endif

// System mode wrapper for IDE compatibility
#ifndef SYSTEM_MODE
#define SYSTEM_MODE(x)
#endif

// A helper for clangd to identify Particle types
#ifndef PARTICLE_IDE_COMPATIBILITY
#define PARTICLE_IDE_COMPATIBILITY 1
namespace Particle {
    class String : public std::string {};
    
    class Serial {
    public:
        template<typename T> 
        static void print(T value) {}
        
        template<typename T>
        static void println(T value) {}
        
        static void begin(int baud) {}
        static bool available() { return false; }
        static int read() { return 0; }
    };
}

// Define Serial as needed
extern Particle::Serial Serial;
extern Particle::Serial Serial1;
extern Particle::Serial Serial2;

#endif // PARTICLE_IDE_COMPATIBILITY

#endif // PARTICLE_MACROS_FIX_H