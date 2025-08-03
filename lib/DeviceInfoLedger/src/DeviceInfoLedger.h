#ifndef __DEVICEINFOLEDGER_H
#define __DEVICEINFOLEDGER_H

// Github: https://github.com/particle-iot/DeviceInfoLedger
// License: Apache 2
// Can use in open or closed source projects, including commercial projects

#include "Particle.h"
#include <atomic>

class DeviceInfoLedgerLogHandler; // Forward declaration

#ifdef UNITTEST
using namespace particle;
#endif

/**
 * This class is a singleton; you do not create one as a global, on the stack, or with new.
 * 
 * From global application setup you must call:
 * DeviceConfigLedger::instance().setup();
 * 
 * Set the options using the withXXX() methods before calling setup().
 */
class DeviceConfigLedger {
public:
public:
    /**
     * @brief Gets the singleton instance of this class, allocating it if necessary
     * 
     * Use DeviceConfigLedger::instance() to instantiate the singleton.
     */
    static DeviceConfigLedger &instance();

    /**
     * @brief Enable default configuration from the cloud using a cloud to device product, owner, or organization ledger.
     * 
     * @param value enable (true) or disable (false)
     * @return DeviceConfigLedger& A reference to this object to chain withXXX() calls, fluent style. 
     * 
     * The class default is disabled; calling this with no parameter will enable it. The default name is "device-info-defaults".
     * 
     * Must be called before setup().
     */
    DeviceConfigLedger &withConfigDefaultLedgerEnabled(bool value = true) { configDefaultLedgerEnabled = value; return *this; };

    /**
     * @brief Enable per-device configuration from the cloud using a cloud to device ledger.
     * 
     * @param value enable (true) or disable (false)
     * @return DeviceConfigLedger& A reference to this object to chain withXXX() calls, fluent style. 
     * 
     * The class default is disabled; calling this with no parameter will enable it. The default name is "device-info-config".
     * 
     * Must be called before setup().
     */
    DeviceConfigLedger &withConfigDeviceLedgerEnabled(bool value = true) { configDeviceLedgerEnabled = value; return *this; };


    /**
     * @brief Set a cloud to device ledger to use to configure default settings.
     * 
     * @param configDefaultLedgerName Name of the ledger to use
     * @return DeviceConfigLedger& A reference to this object to chain withXXX() calls, fluent style. 
     *
     * The default Ledger name is "device-info-defaults" but defaults to disabled. To use the default
     * name, use withConfigDefaultLedgerEnabled() to enable the cloud to device default settings ledger.
     * 
     * Making this call sets the name and also enables it. 
     * 
     * Must be called before setup().
     */
    DeviceConfigLedger &withConfigDefaultLedgerName(const char *configDefaultLedgerName) {
        this->configDefaultLedgerName = configDefaultLedgerName;
        this->configDefaultLedgerEnabled = true;
        return *this;
    }

    /**
     * @brief Set a cloud to device ledger to use to configure device settings.
     * 
     * @param configDeviceLedgerName Name of the ledger to use 
     * @return DeviceConfigLedger& A reference to this object to chain withXXX() calls, fluent style. 
     * 
     * The default Ledger name is "device-info-config" but defaults to disabled. To use the default
     * name, use withConfigDeviceLedgerEnabled() to enable the cloud to device device settings ledger.
     * 
     * Making this call sets the name and also enables it. 
     * 
     * Must be called before setup().
     */
    DeviceConfigLedger &withConfigDeviceLedgerName(const char *configDeviceLedgerName) {
        this->configDeviceLedgerName = configDeviceLedgerName;
        this->configDeviceLedgerEnabled = true;
        return *this;
    }

    /**
     * @brief Use a locally defined configuration (specified with a string of JSON) instead of configDefaultLedger
     * 
     * @param jsonStr The JSON value as a c-string
     * @return DeviceConfigLedger& A reference to this object to chain withXXX() calls, fluent style. 
     * 
     * A key's value is determined by the per-device cloud settings (highest preference), cloud default config, and
     * local config (lowest preference). If you are not using cloud ledgers, then of course the local config
     * is always used since the others will not exist.
     */
    DeviceConfigLedger &withLocalConfig(const char *jsonStr) { localConfig = LedgerData::fromJSON(jsonStr); return *this; };


    /**
     * @brief Adds a function to call when configuration is updated
     * 
     * @param callback 
     * @return DeviceConfigLedger& 
     * 
     * You can use this more than one; the callbacks are a vector. You cannot remove one once added!
     */
    DeviceConfigLedger &withUpdateCallback(std::function<void()> callback) { updateCallbacks.push_back(callback); return *this; };

    /**
     * @brief Call updateCallbacks
     * 
     * This is used internally after the ledger sync callback is called.
     */
    void callUpdateCallbacks();

    /**
     * @brief Perform setup operations; call this from global application setup()
     * 
     * You typically use DeviceConfigLedger::instance().setup();
     * 
     * This must only be called once at boot and not again. You cannot use this to reconfigure settings!
     */
    void setup();


    /**
     * @brief Get a bool (boolean) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @param key Key to read in the top level of the configuration object
     * @param defaultValue Value to be returned if the key does not exist
     * @return true or false depending on the configuration setting or defaultValue.
     * 
     * The defaultValue parameter is optional. It may make more sense to set a local configuration with all of the default values
     * so the defaults are stored in one place, rather than every place the value is accessed.
     */
    bool getConfigBool(const char *key, bool defaultValue = false) const { return getConfigVariant(key, Variant(defaultValue)).toBool(); };

    /**
     * @brief Set a local config setting using a bool
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     */
    bool setLocalConfigBool(const char *key, bool value) { return setLocalConfigVariant(key, Variant(value)); };

    /**
     * @brief Get an int (32-bit signed integer) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @param key Key to read in the top level of the configuration object
     * @param defaultValue Value to be returned if the key does not exist
     * @return true or false depending on the configuration setting or defaultValue.
     * 
     * The defaultValue parameter is optional. It may make more sense to set a local configuration with all of the default values
     * so the defaults are stored in one place, rather than every place the value is accessed.
     */
    int getConfigInt(const char *key, int defaultValue = 0) const { return getConfigVariant(key, Variant(defaultValue)).toInt(); };

    /**
     * @brief Set a local config setting using an int (signed 32-bit integer)
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     */
    bool setLocalConfigInt(const char *key, int value) { return setLocalConfigVariant(key, Variant(value)); };


    /**
     * @brief Get a double (64-bit floating point) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @param key Key to read in the top level of the configuration object
     * @param defaultValue Value to be returned if the key does not exist
     * @return true or false depending on the configuration setting or defaultValue.
     * 
     * The defaultValue parameter is optional. It may make more sense to set a local configuration with all of the default values
     * so the defaults are stored in one place, rather than every place the value is accessed.
     */
    double getConfigDouble(const char *key, double defaultValue = 0.0) const { return getConfigVariant(key, Variant(defaultValue)).toDouble(); };

    /**
     * @brief Set a local config setting using a double (64-bit floating point)
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     */
    bool setLocalConfigDouble(const char *key, double value) { return setLocalConfigVariant(key, Variant(value)); };

    /**
     * @brief Get a String configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @param key Key to read in the top level of the configuration object
     * @param defaultValue Value to be returned if the key does not exist
     * @return true or false depending on the configuration setting or defaultValue.
     * 
     * The defaultValue parameter is optional. It may make more sense to set a local configuration with all of the default values
     * so the defaults are stored in one place, rather than every place the value is accessed.
     */
    String getConfigString(const char *key, const char *defaultValue = "") const { return getConfigVariant(key, Variant(defaultValue)).toString(); };

    /**
     * @brief Set a local config setting using a c-string (null-terminated UTF-8, const char *)
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     */
    bool setLocalConfigString(const char *key, const char *value) { return setLocalConfigVariant(key, Variant(value)); };

    /**
     * @brief Get a configuration setting from local settings or cloud configuration (default or device override)
     *  
     * @param key Top level key in the ledger
     * @param defaultValue Value to be returned if the key does not exist
     * @return Variant Return Variant, see also getConfigBool, getConfigInt, ... that wrap this method
     * 
     * The defaultValue parameter is optional. It may make more sense to set a local configuration with all of the default values
     * so the defaults are stored in one place, rather than every place the value is accessed.
     * 
     * This can be used for both primitive types like bool, int, string, etc. as well as VariantArray and VariantMap.
     */
    Variant getConfigVariant(const char *key, Variant defaultValue = {}) const;

    /**
     * @brief Set a local config setting using a Variant
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     * 
     * See also overloads for specific types such as setLocalConfigBool, setLocalConfigInt, etc.
     */
    bool setLocalConfigVariant(const char *key, Variant value) {
        return localConfig.set(key, value);
    }

    /**
     * @brief Clears the LedgerData (defaultConfig, localConfig, deviceConfig)
     * 
     * It does not reset other settings. You will not normally need to use this, but this
     * method is used by the unit tests.
     */
    void clear();


    /**
     * @brief Update the localConfig ledger data Variant and call the update callbacks
     * 
     * @param data The new data to replace the old data
     * 
     * You will probably never need to call this; it's used internally from the ledger
     * sync callback. It's public because it's used by the unit tests.
     */
    void setLocalConfig(const LedgerData &data) { localConfig = data; callUpdateCallbacks(); };

    /**
     * @brief Update the defaultConfig ledger data Variant and call the update callbacks
     * 
     * @param data The new data to replace the old data
     * 
     * You will probably never need to call this; it's used internally from the ledger
     * sync callback. It's public because it's used by the unit tests.
     */
    void setDefaultConfig(const LedgerData &data) { defaultConfig = data; callUpdateCallbacks(); };

    /**
     * @brief Update the deviceConfig ledger data Variant and call the update callbacks
     * 
     * @param data The new data to replace the old data
     * 
     * You will probably never need to call this; it's used internally from the ledger
     * sync callback. It's public because it's used by the unit tests.
     */
    void setDeviceConfig(const LedgerData &data) { deviceConfig = data; callUpdateCallbacks(); };

protected:

    /**
     * @brief The constructor is protected because the class is a singleton
     * 
     * Use DeviceConfigLedger::instance() to instantiate the singleton.
     */
    DeviceConfigLedger();

    /**
     * @brief The destructor is protected because the class is a singleton and cannot be deleted
     */
    virtual ~DeviceConfigLedger();

    /**
     * This class is a singleton and cannot be copied
     */
    DeviceConfigLedger(const DeviceConfigLedger&) = delete;

    /**
     * This class is a singleton and cannot be copied
     */
    DeviceConfigLedger& operator=(const DeviceConfigLedger&) = delete;


    /**
     * @brief The current default config from the cloud
     */
    LedgerData defaultConfig;

    /**
     * @brief Local configuration (set by JSON or accessors)
     */
    LedgerData localConfig;
    
    /**
     * @brief The current device override config from the cloud
     */
    LedgerData deviceConfig;

    /**
     * @brief Name of the default config ledger
     * 
     * Enable the ledger using withConfigDefaultLedgerEnabled (before setup),
     * Change name using withConfigDefaultLedgerName() (before setup).
     */
    String configDefaultLedgerName = "device-info-defaults";

    /**
     * @brief Name of the device-specific config override ledger
     * 
     * Enable the ledger using withConfigDeviceLedgerEnabled (before setup),
     * Change name using withConfigDeviceLedgerName() (before setup).
     */
    String configDeviceLedgerName = "device-info-config"; 

#ifndef UNITTEST
    /**
     * @brief Ledger default configuration, initialized during setup if enabled
     */
    Ledger configDefaultLedger;

    /**
     * @brief Ledger device-specific override configuration, initialized during setup if enabled
     */
    Ledger configDeviceLedger;
#endif // UNITTEST

    /**
     * @brief Flag if config defaults ledger is enabled
     */
    bool configDefaultLedgerEnabled = false;

    /**
     * @brief Flags if config device-specific overrides are enabled
     */
    bool configDeviceLedgerEnabled = false;

    /**
     * @brief Callback functions to call when configuration is updated
     */
    std::vector<std::function<void()>> updateCallbacks;


    /**
     * @brief Singleton instance of this class
     * 
     * The object pointer to this class is stored here. It's NULL at system boot.
     */
    static DeviceConfigLedger *_instance;
};

/**
 * @brief Wrapper to simplify calling the config methods
 * 
 * Since the config class is a singleton, the syntax for using the method is a little awkward, such as:
 * 
 * DeviceInfoConfig::instance().getConfigString("test")
 * 
 * You make this wrapper a parent class of your class, which makes it possible to just call:
 * 
 * getConfigString("test")
 */
class DeviceConfigWrapper {
public:
    /**
     * @brief Get a bool (boolean) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @param key Key to read in the top level of the configuration object
     * @param defaultValue Value to be returned if the key does not exist
     * @return true or false depending on the configuration setting or defaultValue.
     */
    bool getConfigBool(const char *key, bool defaultValue = false) const { return DeviceConfigLedger::instance().getConfigBool(key, defaultValue); };

    /**
     * @brief Set a local config setting using a bool
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     */
    bool setLocalConfigBool(const char *key, bool value) { return DeviceConfigLedger::instance().setLocalConfigBool(key, value); };

    /**
     * @brief Get an int (32-bit signed integer) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @param key Key to read in the top level of the configuration object
     * @param defaultValue Value to be returned if the key does not exist
     * @return true or false depending on the configuration setting or defaultValue.
     */
    int getConfigInt(const char *key, int defaultValue = 0) const { return DeviceConfigLedger::instance().getConfigInt(key, defaultValue); };

    /**
     * @brief Set a local config setting using an int (signed 32-bit integer)
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     */
    bool setLocalConfigInt(const char *key, int value) { return DeviceConfigLedger::instance().setLocalConfigInt(key, value); };


    /**
     * @brief Get a double (64-bit floating point) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @param key Key to read in the top level of the configuration object
     * @param defaultValue Value to be returned if the key does not exist
     * @return true or false depending on the configuration setting or defaultValue.
     */
    double getConfigDouble(const char *key, double defaultValue = 0.0) const { return DeviceConfigLedger::instance().getConfigDouble(key, defaultValue); };

    /**
     * @brief Set a local config setting using a double (64-bit floating point)
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     */
    bool setLocalConfigDouble(const char *key, double value) { return DeviceConfigLedger::instance().setLocalConfigDouble(key, value); };

    /**
     * @brief Get a String configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @param key Key to read in the top level of the configuration object
     * @param defaultValue Value to be returned if the key does not exist
     * @return true or false depending on the configuration setting or defaultValue.
     */
    String getConfigString(const char *key, const char *defaultValue = "") const { return DeviceConfigLedger::instance().getConfigString(key, defaultValue); };

    /**
     * @brief Set a local config setting using a c-string (null-terminated UTF-8, const char *)
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     */
    bool setLocalConfigString(const char *key, const char *value) { return DeviceConfigLedger::instance().setLocalConfigString(key, value); };

    /**
     * @brief Get a configuration setting from local settings or cloud configuration (default or device override)
     *  
     * @param key Top level key in the ledger
     * @param defaultValue Value to be returned if the key does not exist
     * @return Variant Return Variant, see also getConfigBool, getConfigInt, ... that wrap this method
     */
    Variant getConfigVariant(const char *key, Variant defaultValue = {}) const { return DeviceConfigLedger::instance().getConfigVariant(key, defaultValue);};

    /**
     * @brief Set a local config setting using a Variant
     * 
     * @param key Key to set
     * @param value Value to set to
     * @return true if successfully set, false if not
     * 
     * See also overloads for specific types such as setLocalConfigBool, setLocalConfigInt, etc.
     */
    bool setLocalConfigVariant(const char *key, Variant value) { return DeviceConfigLedger::instance().setLocalConfigVariant(key, value); };

};

/**
 * This class is a singleton; you do not create one as a global, on the stack, or with new.
 * 
 * From global application setup you must call:
 * DeviceInfoLedger::instance().setup();
 * 
 * From global application loop you must call:
 * DeviceInfoLedger::instance().loop();
 */
class DeviceInfoLedger : public DeviceConfigWrapper {
    /**
     * @brief Header for structure in retained memory 
     */
    struct RetainedBufferHeader { // 12 bytes
        uint32_t    magic;
        uint16_t    size;
        uint8_t     headerSize;
        uint8_t     reserved1;
        std::atomic<uint32_t>   offset;
    };

public:
    /**
     * @brief Gets the singleton instance of this class, allocating it if necessary
     * 
     * Use DeviceInfoLedger::instance() to instantiate the singleton.
     */
    static DeviceInfoLedger &instance();


    /**
     * @brief Change the device to cloud ledger that holds the device information.
     * 
     * @param infoLedgerName New name to use.
     * @return DeviceConfigLedger& A reference to this object to chain withXXX() calls, fluent style. 
     * 
     * Default is "device-info". 
     * 
     * Must be called before setup().
     */
    DeviceInfoLedger &withInfoLedgerName(const char *infoLedgerName) {
        this->infoLedgerName = infoLedgerName;
        return *this;
    }

    /**
     * @brief Retained buffer for use mainly to save log information in a circular buffer.
     * 
     * @param retainedBuffer 
     * @param retainedBufferSize 
     * @return DeviceInfoLedger& A reference to this object to chain withXXX() calls, fluent style. 
     * 
     * This log may be useful for debugging problems that cause a device panic, or similar issues that
     * occur before a device reboot.
     * 
     * Because of the way retained memory works on RTL872x devices (P2/Photon 2), this log may be missing
     * the latest data written to the log.
     * 
     * This does not need to match the cloud setting, but the cloud setting can't be used to store
     * more than the size of the retained buffer. Thus you will probably want to make the retained
     * buffer as large as is reasonable for your application.
     * 
     * Must be called before setup().
     */
    DeviceInfoLedger &withRetainedBuffer(uint8_t *retainedBuffer, size_t retainedBufferSize) {
        this->retainedBuffer = retainedBuffer;
        this->retainedBufferSize = retainedBufferSize;
        return *this;
    }

    /**
     * @brief Set the local config log filters from a static array of LogCategoryFilter
     * 
     * @param level 
     * @param filters 
     * @return DeviceInfoLedger& 
     */
    DeviceInfoLedger &withLocalConfigLogLevel(LogLevel level = LOG_LEVEL_INFO, LogCategoryFilters filters = {});

    /**
     * @brief Perform setup operations; call this from global application setup()
     * 
     * You typically use DeviceInfoLedger::instance().setup();
     * 
     * This must only be called once at boot and not again. You cannot use this to reconfigure settings!
     */
    void setup();
    /**
     * @brief Perform application loop operations; call this from global application loop()
     * 
     * You typically use DeviceInfoLedger::instance().loop();
     * 
     * The DeviceInfoLedger needs loop time to process the connectivity changes; the configuration
     * library does not need a loop call.
     */
    void loop();

    /**
     * @brief Get the connectionLog (int) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @return int The size of the connection log in bytes to be included in the ledger. 
     * 
     * The ledger has a maximum size of 16 Kbytes, and could be lower on some devices due to RAM limitations. Be sure to make this
     * field and others small enough that the ledger fits. A normal value would be 2048 bytes.
     */
    int getConfigConnectionLog() const { return getConfigInt("connectionLog", 0); };

    /**
     * @brief Set a local configuration setting for connectionLog (log size). Default is 0 (disabled).
     * 
     * @param value to set
     * @return bool true if successfully set
     */
    DeviceInfoLedger &withLocalConfigConnectionLog(int value) { setLocalConfigInt("connectionLog", value); return *this; };

    /**
     * @brief Get the lastRunLog (int) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @return int The size of the last run log in bytes to be included in the ledger.
     * 
     * The ledger has a maximum size of 16 Kbytes, and could be lower on some devices due to RAM limitations. Be sure to make this
     * field and others small enough that the ledger fits. A normal value would be 2048 bytes.
     * 
     * This is also limited by the amount of retained memory passed to withRetainedBuffer.
     */
    int getConfigLastRunLog() const { return getConfigInt("lastRunLog", 0); };

    /**
     * @brief Set a local configuration setting for lastRunLog (log size). Default is 0 (disabled).
     * 
     * @param value to set
     * @return bool true if successfully set
     */
    DeviceInfoLedger &withLocalConfigLastRunLog(int value) { setLocalConfigInt("lastRunLog", value); return *this; };

    /**
     * @brief Get the logAllConnections (bool) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @return bool 
     */
    bool getConfigLogAllConnections() const { return getConfigBool("logAllConnections", false); };

    /**
     * @brief Set a local configuration setting for logAllConnections. Default is false.
     * 
     * @param value to set
     * @return bool true if successfully set
     */
    DeviceInfoLedger &withLocalConfigLogAllConnections(bool value) { setLocalConfigBool("logAllConnections", value); return *this; };

    /**
     * 
     * @brief Get the includeGeneral (bool) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @return bool 
     */
    bool getConfigIncludeGeneral() const { return getConfigBool("includeGeneral", false); };

    /**
     * @brief Set a local configuration setting for includeGeneral. Default is false.
     * 
     * @param value to set
     * @return bool true if successfully set
     */
    DeviceInfoLedger &withLocalConfigIncludeGeneral(bool value) { setLocalConfigBool("includeGeneral", value); return *this; };


    /**
     * @brief Get the includeDiag (bool) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @return bool 
     */
    bool getConfigIncludeDiag() const { return getConfigBool("includeDiag", false); };

    /**
     * @brief Set a local configuration setting for includeDiag. Default is false.
     * 
     * @param value to set
     * @return bool true if successfully set
     */
    DeviceInfoLedger &withLocalConfigIncludeDiag(bool value) { setLocalConfigBool("includeDiag", value); return *this; };

    /**
     * @brief Get the includeTower (bool) configuration setting from local settings or cloud configuration (default or device override)
     * 
     * @return bool 
     */
    bool getConfigIncludeTower() const { return getConfigBool("includeTower", false); };

    /**
     * @brief Set a local configuration setting for includeTower. Default is false.
     * 
     * @param value to set
     * @return bool true if successfully set
     */
    DeviceInfoLedger &withLocalConfigIncludeTower(bool value) { setLocalConfigBool("includeTower", value); return *this; };

    /**
     * @brief Convert a string like LOG_LEVEL_INFO into its numeric equivalent 
     * 
     * @param levelStr The string (case-sensitive)
     * @return LogLevel numeric log level corresponding to the constant
     * 
     * Note that it really only searches for case-sensitive "INFO", "TRACE", etc. 
     */
    LogLevel stringToLogLevel(const char *levelStr) const;


    /**
     * @brief Convert a log level value (like LOG_LEVEL_INFO) to a string
     * 
     * @param level 
     * @return const char* string string such as "INFO". 
     */
    const char *logLevelToString(LogLevel level) const;

    /**
     * @brief Get the log level settings
     * 
     * @param level 
     * @param filters 
     */
    void getLogLevelFilters(LogLevel &level, LogCategoryFilters &filters) const;

    /**
     * @brief Save the current device information and logs to ledger
     * 
     * @param includeConnLog Include the connection log
     * 
     * This is automatically done on each connection to the cloud, but this method allows it
     * to be done manually on demand. You might call this from a Particle.function, for example.
     */
    void saveDeviceInfo(bool includeConnLog = true);

    /**
     * @brief Called by DeviceInfoLedgerLogHandler to handle log messages
     * 
     * @param c A character written to the logger
     */
    void write(uint8_t c);

    /**
     * @brief Magic bytes at the beginning of the RetainedBufferHeader
     * 
     * This is used to see if the structure has been initialized.
     */
    static const uint32_t retainedMagicBytes = 0xde8e46cc;

protected:

    /**
     * @brief The constructor is protected because the class is a singleton
     * 
     * Use DeviceInfoLedger::instance() to instantiate the singleton.
     */
    DeviceInfoLedger();

    /**
     * @brief The destructor is protected because the class is a singleton and cannot be deleted
     */
    virtual ~DeviceInfoLedger();

    /**
     * This class is a singleton and cannot be copied
     */
    DeviceInfoLedger(const DeviceInfoLedger&) = delete;

    /**
     * This class is a singleton and cannot be copied
     */
    DeviceInfoLedger& operator=(const DeviceInfoLedger&) = delete;



    /**
     * @brief Configure the log handler with the current settings in logLevel and logFilter
     */
    void configureLogHandler();

    /**
     * @brief Called on the any cloud connection completed
     */
    virtual void onCloudConnection();

    /**
     * @brief Called at setup and when the configuration is updated
     */
    void updateConfig();

    /**
     * @brief Name of the info ledger
     * 
     * Change name using withInfoLedgerName() (before setup).
     */
    String infoLedgerName = "device-info";


    /**
     * @brief Retained buffer for lastRunLog and other purposes. 
     * 
     * Set using withRetainedBuffer() before setup.
     */
    uint8_t *retainedBuffer = nullptr;

    /**
     * @brief Retained buffer size for lastRunLog and other purposes. 
     * 
     * Set using withRetainedBuffer() before setup.
     */
    size_t retainedBufferSize = 0;

    /**
     * @brief lastRunLog, set during setup(), cleared when uploaded
     */
    char *lastRunLog = nullptr;

    /**
     * @brief Buffer for connectionLog, allocated during updateConfig()
     */
    uint8_t *connectionLogBuffer = nullptr;

    /**
     * @brief Size of buffer for connectionLog, set during updateConfig()
     */
    size_t connectionLogSize = 0;

    /**
     * @brief Offset for writing to connectionLog. Written in write().
     */
    std::atomic<uint32_t> connectionLogOffset;

    /**
     * @brief Whether to save data to the connection log
     */
    bool writeToConnectionLog = true;

    /**
     * @brief Internal state, true if Cellular.ready() returned true
     */
    bool isCellularConnected = false;

    /**
     * @brief Internal state, true if WiFi.ready() returned true
     */
    bool isWiFiConnected = false;

    /**
     * @brief Internal state, true if Network.ready() returned true
     */
    bool isNetworkConnected = false;

    /**
     * @brief Internal state, true if Particle.connected() returned true
     */
    bool isCloudConnected = false;

    /**
     * @brief Internal state, number of times Particle.connected() has transitioned to trye
     */
    int connectionCount = 0;

    /**
     * @brief Internal state, true if setup has been called and completed
     */
    bool setupComplete = false;

    /**
     * @brief Convenience pointer into retainedBuffer. Set during setup.
     */
    RetainedBufferHeader *retainedHdr = nullptr;

    /**
     * @brief Convenience pointer into retainedBuffer for retained log data. Set during setup.
     */
    uint8_t *retainedData = nullptr;

    /**
     * @brief Amount of spaces for retained log data. Set during setup.
     */
    size_t retainedDataSize = 0;

#ifndef UNITTEST
    /**
     * @brief Reset reason, saved at the beginning of setup
     */
    int resetReason = RESET_REASON_NONE;

    /**
     * @brief Reset reason data, saved at the beginning of setup if RESET_REASON_USER.
     */
    uint32_t resetReasonData = 0;

    /**
     * @brief Ledger object for device information, initialized during setup
     */
    Ledger infoLedger;    
#endif // UNITTEST

    /**
     * @brief The StreamLogHandler object
     * 
     * Allocated during setup() and when the configuration changes.
     */
    DeviceInfoLedgerLogHandler *logHandler = nullptr;

    /**
     * @brief Singleton instance of this class
     * 
     * The object pointer to this class is stored here. It's NULL at system boot.
     */
    static DeviceInfoLedger *_instance;

};

#ifndef UNITTEST
/**
 * @brief Log handler class, used internally
 */
class DeviceInfoLedgerLogHandler : public StreamLogHandler, public Print {
public:
    /**
     * @brief Construct log handler object, allocated during updateConfig
     * 
     * @param level 
     * @param filters 
     */
    DeviceInfoLedgerLogHandler(LogLevel level, LogCategoryFilters filters);

    /**
     * @brief Deleted when a new log handler is created
     */
    virtual ~DeviceInfoLedgerLogHandler();
    
    /**
     * @brief Virtual override in class Print for the StreamLogHandler to write data to the log
     * 
     * Calls write() in the DeviceInfoLedger singleton instance
     */
    virtual size_t write(uint8_t);
};
#endif // UNITTEST


#endif  /* __DEVICEINFOLEDGER_H */