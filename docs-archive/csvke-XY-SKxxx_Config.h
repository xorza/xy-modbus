// source_url: https://github.com/csvke/XY-SK120-Modbus-RTU-TTL/blob/main/include/XY-SKxxx_Config.h
// fetched: 2026-04-27

#ifndef XY_SKXXX_CONFIG_H
#define XY_SKXXX_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>

// Default hardware settings
#define DEFAULT_MODBUS_RX_PIN D7     // Default RX pin (D7 on XIAO ESP32S3)
#define DEFAULT_MODBUS_TX_PIN D6     // Default TX pin (D6 on XIAO ESP32S3)
#define DEFAULT_MODBUS_SLAVE_ID 1    // Default Modbus slave ID
#define DEFAULT_MODBUS_BAUD_RATE 115200  // Default baud rate

// NVS namespace for storing settings
#define PREFS_NAMESPACE "xysk120"

// Configuration structure
struct XYModbusConfig {
    uint8_t rxPin;          // RX pin for Modbus communication
    uint8_t txPin;          // TX pin for Modbus communication
    uint8_t slaveId;        // Modbus slave ID
    uint32_t baudRate;      // Baud rate for Modbus communication
    
    // Default constructor uses default values
    XYModbusConfig() :
        rxPin(DEFAULT_MODBUS_RX_PIN),
        txPin(DEFAULT_MODBUS_TX_PIN),
        slaveId(DEFAULT_MODBUS_SLAVE_ID),
        baudRate(DEFAULT_MODBUS_BAUD_RATE)
    {}
};

// Class to manage configuration settings
class XYConfigManager {
public:
    // Initialize the configuration manager
    static bool begin() {
        return _preferences.begin(PREFS_NAMESPACE, false);
    }
    
    // End the configuration manager session
    static void end() {
        _preferences.end();
    }
    
    // Load configuration from NVS
    static XYModbusConfig loadConfig() {
        XYModbusConfig config;
        
        // If a key exists in NVS, read it; otherwise, use the default value
        if (_preferences.isKey("rxPin")) {
            config.rxPin = _preferences.getUChar("rxPin", DEFAULT_MODBUS_RX_PIN);
        }
        
        if (_preferences.isKey("txPin")) {
            config.txPin = _preferences.getUChar("txPin", DEFAULT_MODBUS_TX_PIN);
        }
        
        if (_preferences.isKey("slaveId")) {
            config.slaveId = _preferences.getUChar("slaveId", DEFAULT_MODBUS_SLAVE_ID);
        }
        
        if (_preferences.isKey("baudRate")) {
            config.baudRate = _preferences.getULong("baudRate", DEFAULT_MODBUS_BAUD_RATE);
        }
        
        return config;
    }
    
    // Save configuration to NVS
    static bool saveConfig(const XYModbusConfig& config) {
        _preferences.putUChar("rxPin", config.rxPin);
        _preferences.putUChar("txPin", config.txPin);
        _preferences.putUChar("slaveId", config.slaveId);
        _preferences.putULong("baudRate", config.baudRate);
        
        return true;
    }
    
    // Reset configuration to defaults
    static bool resetConfig() {
        XYModbusConfig defaultConfig;
        return saveConfig(defaultConfig);
    }
    
    // Check if configuration exists in NVS
    static bool configExists() {
        return _preferences.isKey("rxPin") || 
               _preferences.isKey("txPin") || 
               _preferences.isKey("slaveId") || 
               _preferences.isKey("baudRate");
    }

private:
    // Static member only declared here, defined elsewhere
    static Preferences _preferences;
};

#endif // XY_SKXXX_CONFIG_H
