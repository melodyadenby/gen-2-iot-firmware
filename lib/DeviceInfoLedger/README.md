# DeviceInfoLedger

This library perform two separate but related things using two separate classes:

- Store cloud-based configuration for devices (DeviceConfigLedger)
- Store Particle device log data and device information in Ledger (DeviceInfoLedger)

You can use the cloud-based configuration without using the DeviceInfoLedger, if desired. 
You can also use the DeviceInfoLedger with a local configuration, not using the remote configuration option.

## Configuration

This library supports three levels of configuration, all of which are optional:

- Local (on-device) configuration
- Product, owner, or organization cloud-to-device ledger
- Device-specific cloud-to-device ledger

Using Ledger, this library supports a default configuration using a cloud-to-device ledger (scoped to a product, owner, or organization). This is good for making settings cloud-based instead of on-device, so you can change fleet settings without having to update code.

It also supports an optional per-device configuration using a cloud-to-device device ledger (scoped to a device). The per-device configuration overrides specific values, making it easy to just change one setting and leave the others as the default values.

Configuration support key (string) and value pairs, using types that can be represented in JSON and `Variant` including:

- boolean values (`bool` in C++)
- integer number values (`int`, `unsigned`, `int64_t`, `uint64_t`)
- floating point number values (`double`)
- string values (`String`)

For example say this is the product default configuration:

```json
{
    "a": 123,
    "b": false,
    "c": "test"
}
```

If you set the device configuration to, the value of `b` will be overridden but the values of `a` and `c` will remain the product default.

```json
{
    "b": true
}
```


Additionally, the top level can also include one-level deep of object and array types.

When an object is contained in the configuration, all values for the same key are merged so you can easily override
specific sub-keys. With this product/owner/organization setting:

```json
{
    "x": {
        "a": 123,
        "b": false,
        "c": "test"
    }
}
```

And this per-device setting:

```json
{
    "x": {
        "a": 456,
        "d": 789
    }
}
```

The configuration will act as follows:

```json
{
    "x": {
        "a": 456,
        "b": false,
        "c": "test",
        "d": 789
    }
}
```

When an array is contained in the configuration, all array values for the key are appended. For example:

```json
{
    "a": [1, 2]
}
```

With a device-specific override:

```json
{
    "a": [2, 3]
}
```

The values are all appended in the order local, product/owner/organization, device. Duplicates are not removed!

```json
{
    "a": [1, 2, 2, 3]
}
```

Note that only one level of nested object or array is supported! If you nest two objects, the inner objects are not merged and will simply be replaced like a primitive value.

Using cloud-based configuration is optional if you only want to use device log data and device information. These settings can be 
configured locally on device, if desired. However, you may want to use both so you can control the log settings remotely from
the cloud-side.

The [full DeviceInfoLedger API documentation](https://particle-iot.github.io/DeviceInfoLedger/class_device_config_ledger.html)
can be found at the link, or in the docs directory of this repository.


## Device information ledger

The following JSON structure is used for default cloud configuration, and device-specific overrides, if those features are enabled.

```json
{
    "lastRunLog": 1024,
    "connectionLog": 2048,
    "logAllConnections": false,
    "includeGeneral": true,
    "includeDiag": true,
    "includeTower": true,
    "logLevel": "INFO",
    "logFilters": [],
}
```

Cloud-based device overrides can specify only the fields that need to be changed from the default configuration.

You can also set the settings locally, as a default, or in lieu of the cloud configuration. Local settings for 
device information ledger can be done using JSON, or by calling individual methods like `withLastRunLog()`.

The [full DeviceInfoLedger API documentation](https://particle-iot.github.io/DeviceInfoLedger/class_device_config_ledger.html)
can be found at the link, or in the docs directory of this repository.

### Detailed description of fields

#### lastRunLog (int)

The last run log is the latest log messages prior to the most recent reboot. This can be useful for troubleshooting device resets.

Value is the size of the text to include in the ledger. Must fit within the available space in the ledger, which is up to 16K but
can be lower on some platforms.

The last run log is stored in retained memory. This is specified in the application source because it's allocated by the compiler,
so the actual run log will be the lesser of lastRunLog and the value stored in the source.

On RTL872x devices (P2, Photon 2, M-SoM), the most recent log entries may not be available if a system panic has occurred.

#### connectionLog (int)

The connection log is the most recent messages from boot until successfully cloud connected on the first connection after
reboot (default) or after every cloud disconnect (if logAllConnections is true). Note that logging every disconnection
may result in a large number of ledger synchronization is the device is frequently disconnecting from the cloud.

#### logAllConnections (bool)

#### includeGeneral (bool)

Include general information. Default: false.

This currently only includes `sysver`, the Device OS version.

#### includeDiag (bool)

Include the device vitals information in ledger. Default: false

#### includeTower (bool)

Includes cellular tower information for the connected tower. Default: false

This is stored in the `tower` key as an object with the following values:

| Field | Description |
| :---- | :--- |
| `mcc` | Mobile country code |
| `mnc` | Mobile network code |
| `cid` | Cell identifier |
| `lac` | Location area code |


#### logLevel (string)

| Constant           | Configuration String |
| :----------------- | :------- |
| `LOG_LEVEL_ALL`    | `ALL`    |
| `LOG_LEVEL_TRACE`  | `TRACE`  | 
| `LOG_LEVEL_INFO`   | `INFO`   |
| `LOG_LEVEL_WARN`   | `WARN`   | 
| `LOG_LEVEL_ERROR`  | `ERROR`  |
| `LOG_LEVEL_PANIC`  | `PANIC`  |
| `LOG_LEVEL_NONE`   | `NONE`   |


#### logFilters (object)

This is a JSON representation of a logging configuration for `LogCategoryFilter`. It is an object whose
key is the category and value is the logging level.

For example, given this code:

```cpp
SerialLogHandler logHandler(LOG_LEVEL_WARN, { // Logging level for non-application messages
    { "app", LOG_LEVEL_INFO }, // Default logging level for all application messages
    { "app.network", LOG_LEVEL_TRACE } // Logging level for networking messages
});
```

An equivalent JSON configuration would be:

```json
{
    "logLevel": "LOG_LEVEL_WARN",
    "logFilters": {
        "app": "INFO",
        "app.network": "TRACE"
    }
}
```

You can also configure log filters locally using the same style as Device OS log handlers:

```cpp
DeviceInfoLedger::instance().withLocalConfigLogLevel(LOG_LEVEL_INFO, {
    { "app.devinfo", LOG_LEVEL_TRACE }
});
```

## Version history

### 0.0.4 (2025-06-01)

- Added an example 3-tester to test sending logs after waking up from sleep

### 0.0.3 (2025-03-10)

- Added a `saveDeviceInfo(bool includeConnLog = true)` method that can be used force an upload at a time other than on cloud connection. You might do this from a Particle.function for example, to allow the logs be uploaded on demand.

### 0.0.2 (2025-01-06)

- Fixed a bug where the log handler would not be initialized if the ledger was already cached locally when using a cloud configuration ledger.
