#include "config.h"
#include "Particle.h"

// Global configuration variables that need storage
// Most configuration is handled via preprocessor defines in config.h
// This file contains any runtime configuration management if needed

// Get current environment as string for debugging
const char *getCurrentEnvironment()
{
#if ENV == ENV_PROD
  return "PRODUCTION";
#elif ENV == ENV_QA
  return "QA";
#elif ENV == ENV_LOCAL
  return "LOCAL";
#else
  return "DEVELOPMENT";
#endif
}

// Get build info string
String getBuildInfo()
{
  String info = "Build: ";
  info += BUILD_VERSION;
  info += ", Env: ";
  info += getCurrentEnvironment();
  return info;
}
