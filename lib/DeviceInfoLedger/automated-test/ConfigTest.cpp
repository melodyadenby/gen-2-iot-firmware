#include <stdio.h>

#include "DeviceInfoLedger.h"

void runUnitTests();

int main(int argc, char *argv[]) {
    runUnitTests();
    return 0;
}

#define assertInt(exp, val) \
    if (val != exp) { printf("exp=%d val=%d\n", (int)exp, (int)val); assert(false); }

#define assertDouble(exp, val, tol) \
    if (val < (exp - tol) || val > (exp + tol)) { printf("exp=%lf val=%lf\n", (double)exp, (double)val); assert(false); }

#define assertString(exp, val) \
    if (strcmp(val, exp) != 0) { printf("exp=%s val=%s\n", exp, val); assert(false); }

const char jsonConfig1[] = 
"{"
    "\"lastRunLog\": 1024,"
    "\"connectionLog\": 2048,"
    "\"includeGeneral\": true,"
    "\"includeDiag\": false,"
    "\"includeTower\": false,"
    "\"logLevel\": \"INFO\","
    "\"logFilters\": {}"
"}";

const char jsonConfig2[] = 
"{"
    "\"lastRunLog\": 4096,"
    "\"connectionLog\": 2048,"
    "\"logLevel\": \"TRACE\""
"}";

const char jsonConfig3[] = 
"{"
    "\"lastRunLog\": 1024,"
    "\"connectionLog\": 1024,"
    "\"includeGeneral\": true,"
    "\"includeDiag\": false,"
    "\"includeTower\": false,"
    "\"logLevel\": \"ERROR\","
    "\"logFilters\": {}"
"}";

const char jsonConfig4[] = 
"{"
    "\"lastRunLog\": 1024,"
    "\"connectionLog\": 4096,"
    "\"includeTower\": true,"
    "\"logLevel\": \"INFO\","
"}";

/*
{
    "logLevel": "INFO",
    "logFilters": {
        "app.devinfo": "TRACE"
    }
}
*/
const char jsonConfig5[] = 
"{"
    "\"logLevel\": \"INFO\","
    "\"logFilters\": {"
        "\"app.devinfo\": \"TRACE\""
    "}"
"}";

const char jsonConfig6[] = 
"{"
    "\"logLevel\": \"INFO\","
    "\"logFilters\": {"
        "\"app.devinfo\": \"INFO\""
    "}"
"}";

const char jsonConfig7[] = 
"{"
    "\"logFilters\": {"
        "\"app.test\": \"TRACE\""
    "}"
"}";



void runUnitTests() {
    // Local unit tests only used off-device 

    // Default values
    {
        DeviceConfigLedger::instance().clear();

        assertInt(0, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(0, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        LogLevel level;
        LogCategoryFilters filters;
        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_NONE, level);
        assertInt(0, filters.size());
    }
    
    // Set from JSON
    {
        DeviceConfigLedger::instance().clear();

        DeviceConfigLedger::instance().withLocalConfig(jsonConfig1);

        assertInt(1024, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        LogLevel level;
        LogCategoryFilters filters;
        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(0, filters.size());

        // Test resetting value
        DeviceInfoLedger::instance().withLocalConfigConnectionLog(4096);
        assertInt(4096, DeviceInfoLedger::instance().getConfigConnectionLog());
    }

    // Set from code
    {
        DeviceConfigLedger::instance().clear();

        DeviceInfoLedger::instance().withLocalConfigLastRunLog(1024);
        DeviceInfoLedger::instance().withLocalConfigConnectionLog(2048);
        DeviceInfoLedger::instance().withLocalConfigIncludeGeneral(true);
        DeviceInfoLedger::instance().withLocalConfigLogLevel(LOG_LEVEL_INFO);


        assertInt(1024, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        LogLevel level;
        LogCategoryFilters filters;
        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(0, filters.size());
    }

    // Cloud defaults, no local config
    {
        DeviceConfigLedger::instance().clear();

        DeviceConfigLedger::instance().setDefaultConfig(LedgerData::fromJSON(jsonConfig1));

        assertInt(1024, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        LogLevel level;
        LogCategoryFilters filters;
        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(0, filters.size());
    }

    // Cloud default overrides local config
    {
        DeviceConfigLedger::instance().clear();


        DeviceConfigLedger::instance().withLocalConfig(jsonConfig1);

        assertInt(1024, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        LogLevel level;
        LogCategoryFilters filters;
        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(0, filters.size());

        DeviceConfigLedger::instance().setDefaultConfig(LedgerData::fromJSON(jsonConfig2));

        assertInt(4096, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_TRACE, level);
        assertInt(0, filters.size());
    }

    // Device overrides local config
    {
        DeviceConfigLedger::instance().clear();

        DeviceConfigLedger::instance().withLocalConfig(jsonConfig1);

        assertInt(1024, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        LogLevel level;
        LogCategoryFilters filters;
        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(0, filters.size());

        DeviceConfigLedger::instance().setDeviceConfig(LedgerData::fromJSON(jsonConfig2));

        assertInt(4096, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_TRACE, level);
        assertInt(0, filters.size());
    }    

    // Device config overrides cloud config
    {
        DeviceConfigLedger::instance().clear();

        DeviceConfigLedger::instance().setDefaultConfig(LedgerData::fromJSON(jsonConfig1));

        assertInt(1024, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        LogLevel level;
        LogCategoryFilters filters;
        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(0, filters.size());

        DeviceConfigLedger::instance().setDeviceConfig(LedgerData::fromJSON(jsonConfig2));

        assertInt(4096, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_TRACE, level);
        assertInt(0, filters.size());
    }    

    // Three levels of config
    {
        DeviceConfigLedger::instance().clear();

        LogLevel level;
        LogCategoryFilters filters;

        DeviceConfigLedger::instance().withLocalConfig(jsonConfig3);

        assertInt(1024, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(1024, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeTower());

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_ERROR, level);
        assertInt(0, filters.size());


        DeviceConfigLedger::instance().setDefaultConfig(LedgerData::fromJSON(jsonConfig1));

        assertInt(1024, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(2048, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeTower());

        DeviceConfigLedger::instance().setDeviceConfig(LedgerData::fromJSON(jsonConfig4));

        assertInt(1024, DeviceInfoLedger::instance().getConfigLastRunLog());
        assertInt(4096, DeviceInfoLedger::instance().getConfigConnectionLog());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeGeneral());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());
        assertInt(false, DeviceInfoLedger::instance().getConfigIncludeDiag());
        assertInt(true, DeviceInfoLedger::instance().getConfigIncludeTower());

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(0, filters.size());

    }

    // Filters from JSON
    {
        DeviceConfigLedger::instance().clear();
        LogLevel level;
        LogCategoryFilters filters;

        DeviceConfigLedger::instance().withLocalConfig(jsonConfig5);

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(1, filters.size());

        assertString("app.devinfo", filters.at(0).category());
        assertInt(LOG_LEVEL_TRACE, filters.at(0).level());
    }

    // Filters from code
    {
        DeviceConfigLedger::instance().clear();
        LogLevel level;
        LogCategoryFilters filters;

        DeviceInfoLedger::instance().withLocalConfigLogLevel(LOG_LEVEL_INFO, {
            { "app.devinfo", LOG_LEVEL_TRACE }
        });

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(1, filters.size());

        assertString("app.devinfo", filters.at(0).category());
        assertInt(LOG_LEVEL_TRACE, filters.at(0).level());
    }

    // Log filter merging - local config (JSON) + device config (no overlap)
    {
        DeviceConfigLedger::instance().clear();
        LogLevel level;
        LogCategoryFilters filters;

        DeviceConfigLedger::instance().withLocalConfig(jsonConfig6);

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(1, filters.size());

        assertString("app.devinfo", filters.at(0).category());
        assertInt(LOG_LEVEL_INFO, filters.at(0).level());

        DeviceConfigLedger::instance().setDefaultConfig(LedgerData::fromJSON(jsonConfig7));

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(2, filters.size());

        assertString("app.devinfo", filters.at(0).category());
        assertInt(LOG_LEVEL_INFO, filters.at(0).level());

        assertString("app.test", filters.at(1).category());
        assertInt(LOG_LEVEL_TRACE, filters.at(1).level());

    }

    // Log filter merging - local config (JSON) + device config override category
    {
        DeviceConfigLedger::instance().clear();
        LogLevel level;
        LogCategoryFilters filters;

        DeviceConfigLedger::instance().withLocalConfig(jsonConfig6);

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(1, filters.size());

        assertString("app.devinfo", filters.at(0).category());
        assertInt(LOG_LEVEL_INFO, filters.at(0).level());

        DeviceConfigLedger::instance().setDeviceConfig(LedgerData::fromJSON(jsonConfig5));

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(1, filters.size());

        assertString("app.devinfo", filters.at(0).category());
        assertInt(LOG_LEVEL_TRACE, filters.at(0).level());

    }

    // Filters array in device config overrides local    
    {
        DeviceConfigLedger::instance().clear();
        LogLevel level;
        LogCategoryFilters filters;

        DeviceConfigLedger::instance().withLocalConfig(jsonConfig6);

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(1, filters.size());

        assertInt(LOG_LEVEL_INFO, filters.at(0).level());
        assertString("app.devinfo", filters.at(0).category());


        DeviceConfigLedger::instance().setDeviceConfig(LedgerData::fromJSON(jsonConfig5));

        DeviceInfoLedger::instance().getLogLevelFilters(level, filters);
        assertInt(LOG_LEVEL_INFO, level);
        assertInt(1, filters.size());

        assertString("app.devinfo", filters.at(0).category());
        assertInt(LOG_LEVEL_TRACE, filters.at(0).level());
    }

    // Arbitrary configuration

    const char jsonConfig100[] = 
    "{"
        "\"a\": 123,"
        "\"b\": true,"
        "\"c\": \"test\""
    "}";
    const char jsonConfig101[] = 
    "{"
        "\"a\": 456"
    "}";

    {
        DeviceConfigLedger::instance().clear();

        DeviceConfigLedger::instance().setDefaultConfig(LedgerData::fromJSON(jsonConfig100));

        assertInt(123, DeviceConfigLedger::instance().getConfigInt("a"));
        assertInt(true, DeviceConfigLedger::instance().getConfigBool("b"));
        assertString("test", DeviceConfigLedger::instance().getConfigString("c").c_str());


        DeviceConfigLedger::instance().setDeviceConfig(LedgerData::fromJSON(jsonConfig101));

        assertInt(456, DeviceConfigLedger::instance().getConfigInt("a"));
    }

    // Array merge
    const char jsonConfig110[] = 
    "{"
        "\"a\": [1, 2]"
    "}";
    const char jsonConfig111[] = 
    "{"
        "\"a\": [2, 3]"
    "}";


    {
        DeviceConfigLedger::instance().clear();
        Variant a;

        DeviceConfigLedger::instance().setDefaultConfig(LedgerData::fromJSON(jsonConfig110));

        a = DeviceConfigLedger::instance().getConfigVariant("a");
        assertInt(2, a.size());
        assertInt(1, a.at(0).toInt());
        assertInt(2, a.at(1).toInt());


       DeviceConfigLedger::instance().setDeviceConfig(LedgerData::fromJSON(jsonConfig111));

        a = DeviceConfigLedger::instance().getConfigVariant("a");
        assertInt(4, a.size());
        assertInt(1, a.at(0).toInt());
        assertInt(2, a.at(1).toInt());
        assertInt(2, a.at(2).toInt());
        assertInt(3, a.at(3).toInt());
    }

    // Object merge

    const char jsonConfig120[] = 
    "{"
        "\"x\": {"
            "\"a\": 123,"
            "\"b\": true,"
            "\"c\": \"test\""
        "}"
    "}";
    const char jsonConfig121[] = 
    "{"
        "\"x\": {"
            "\"a\": 456,"
            "\"d\": 789"
        "}"
    "}";

    {
        DeviceConfigLedger::instance().clear();
        Variant x;

        DeviceConfigLedger::instance().setDefaultConfig(LedgerData::fromJSON(jsonConfig120));

        x = DeviceConfigLedger::instance().getConfigVariant("x");
        assertInt(123, x.get("a").toInt());
        assertInt(true, x.get("b").toBool());
        assertString("test", x.get("c").toString().c_str());


        DeviceConfigLedger::instance().setDeviceConfig(LedgerData::fromJSON(jsonConfig121));

        x = DeviceConfigLedger::instance().getConfigVariant("x");
        assertInt(456, x.get("a").toInt());
    }


}




