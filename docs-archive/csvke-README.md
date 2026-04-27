---
source_url: https://github.com/csvke/XY-SK120-Modbus-RTU-TTL/blob/main/README.md
fetched: 2026-04-27
---

# XY-SK120 Power Supply Control

![Demo](documentation/demo.jpg)

![Demo](documentation/demo.gif)

This project allows you to control an XY-SK120 power supply (and compatible models) over Modbus RTU using a Seeed XIAO ESP32S3.

## Features

- Serial monitor control interface
- Memory group management
- Direct register access for debugging
- Web interface (coming soon)

## Hardware Setup

- Connect the XIAO ESP32S3 to the XY-SK120 using the TTL interface:
  - XIAO TX pin → XY-SK120 RX pin
  - XIAO RX pin → XY-SK120 TX pin
  - XIAO GND → XY-SK120 GND

## Serial Monitor Commands

### Basic Commands

- `on` - Turn output ON
- `off` - Turn output OFF
- `set V I` - Set voltage (V) and current (I)
- `status` - Display current status
- `info` - Display device information
- `config` - Display current configuration
- `save` - Save current config to NVS
- `reset` - Reset config to defaults
- `help` - Show help message

### Memory Group Commands

- `mem N` - Display memory group N (0-9)
- `call N` - Call memory group N (1-9) to active memory
- `save2mem N` - Save current settings to memory group N (1-9)
- `setmem N param value` - Set specific parameter in memory group N
  - Parameters: v(voltage), i(current), p(power), ovp, ocp, opp, oah, owh, uvp, ucp

### Debug Commands

The project supports direct register access for debugging and advanced usage:

- `read addr count` - Read 'count' registers starting at address 'addr'
- `write addr value` - Write 'value' to register at address 'addr'
- `writes addr v1 v2 ...` - Write multiple values to consecutive registers

Examples:
- `read 0x0000 1` - Read the voltage setting register (result shows different scaling factors)
- `write 0x0000 1250` - Set voltage to 12.50V (1250/100)
- `read 0x0002 3` - Read output voltage, current, and power

Addresses can be provided in decimal or hexadecimal (with 0x prefix).

## Register Map

The power supply uses the following register map (partial list):

| Address | Description | Unit | Scaling | R/W |
|---------|-------------|------|---------|-----|
| 0x0000  | Voltage setting | V | /100 | R/W |
| 0x0001  | Current setting | A | /1000 | R/W |
| 0x0002  | Output voltage | V | /100 | R |
| 0x0003  | Output current | A | /1000 | R |
| 0x0004  | Output power | W | /100 | R |
| 0x0012  | Output on/off | 0/1 | - | R/W |

For a more complete register map, see the XY-SKxxx library documentation.

## Building

This project uses PlatformIO. To build and upload:

```
pio run -t upload
```

## Building the CSS

The project uses Tailwind CSS for styling. To build the CSS:

1. Make sure you have Node.js and npm installed
2. Run the build script:
   ```bash
   # Make the script executable (first time only)
   chmod +x build-css.sh
   
   # Run the build script
   ./build-css.sh
   ```

3. For development with auto-reloading:
   ```bash
   npm run watch:css
   ```

This will generate the `data/css/tailwind.css` file used by the application.

## Developer Guide: Adding Features to the Library

This guide outlines the complete process of discovering, implementing, and exposing new features in the XY-SKxxx library.

### 1. Feature Discovery Process

#### Using Debug Mode to Scan Registers

The first step in adding a new feature is discovering which Modbus registers control it:

1. **Access the Debug Menu via Serial Monitor**:
   ```
   menu
   5
   ```

2. **Scan Register Ranges**:
   ```
   scan 0x0000 0x0030
   ```
   This examines registers in blocks. For example, configuration registers are typically found in 0x0000-0x0030, while protection settings often appear in 0x0050-0x0080.

3. **Record Initial Values**:
   Take note of all current register values before making any changes.

#### Verify Register Function via OSD Testing

1. **Change Settings via Device Screen**:
   - Modify the setting you're investigating through the device's physical interface
   - Make small, known changes (e.g., change MPPT from OFF to ON)

2. **Rescan Registers to Detect Changes**:
   ```
   scan 0x0000 0x0030
   ```
   Compare with your initial values to identify which register changed.

3. **Validate with Direct Register Read/Write**:
   ```
   read 0x001F   # Example: reading MPPT enable register
   ```

4. **Test Writing to Register**:
   ```
   write 0x001F 1   # Example: enabling MPPT
   ```
   Verify the change takes effect on the device display.

### 2. Implementing the Feature in the Library

#### Step 1: Add Register Definitions

Update `XY-SKxxx.h` with the new register definitions:

```cpp
// Add to register definitions section
#define REG_MPPT_ENABLE 0x001F  // MPPT enable/disable, 2 bytes, 0 decimal places, unit: 0/1
#define REG_MPPT_THRESHOLD 0x0020 // MPPT threshold percentage, 2 bytes, 2 decimal places, unit: ratio (0.00-1.00)
```

#### Step 2: Add Class Member Variables

Add private member variables to store cached values:

```cpp
// Add to private section of XY_SKxxx class
bool _mpptEnabled;         // MPPT enable state cache
float _mpptThreshold;      // MPPT threshold cache
```

#### Step 3: Declare Interface Methods

Add method declarations to the public interface:

```cpp
// Add to public section of XY_SKxxx class
bool setMPPTEnable(bool enabled);
bool getMPPTEnable(bool &enabled);
bool setMPPTThreshold(float threshold);
bool getMPPTThreshold(float &threshold);
```

#### Step 4: Implement Methods

Create implementation in an appropriate file (e.g., `XY-SKxxx-settings.cpp`):

```cpp
/**
 * Enable or disable MPPT (Maximum Power Point Tracking)
 * 
 * @param enabled true to enable, false to disable
 * @return true if successful
 */
bool XY_SKxxx::setMPPTEnable(bool enabled) {
    waitForSilentInterval();
    uint8_t result = modbus.writeSingleRegister(REG_MPPT_ENABLE, enabled ? 1 : 0);
    _lastCommsTime = millis();
    if (result == modbus.ku8MBSuccess) {
        _mpptEnabled = enabled;  // Update cache
        return true;
    }
    return false;
}

// Implement remaining methods...
```

### 3. Cache Management

#### Step 1: Update Cache Methods

Modify the cache update methods (e.g., in `XY-SKxxx-cache.cpp`):

```cpp
bool XY_SKxxx::updateCalibrationSettings(bool force) {
    // ...existing code...
    
    // Read MPPT enable state
    delay(_silentIntervalTime * 2);
    result = modbus.readHoldingRegisters(REG_MPPT_ENABLE, 1);
    if (result == modbus.ku8MBSuccess) {
        _mpptEnabled = (modbus.getResponseBuffer(0) != 0);
    }
    
    // Read MPPT threshold
    delay(_silentIntervalTime * 2);
    result = modbus.readHoldingRegisters(REG_MPPT_THRESHOLD, 1);
    if (result == modbus.ku8MBSuccess) {
        _mpptThreshold = modbus.getResponseBuffer(0) / 100.0f;
    }
    
    // ...existing code...
}
```

#### Step 2: Use Cache in Getter Methods

Make getters use the cache:

```cpp
bool XY_SKxxx::getMPPTEnable(bool &enabled) {
    // Try from cache first
    if (updateCalibrationSettings(false)) {
        enabled = _mpptEnabled;
        return true;
    }
    
    // If cache failed, read directly
    // ...direct reading code...
}
```

### 4. User Interface Integration

#### Step 1: Add to Menu Display

Update the relevant menu display function (e.g., `menu_settings.cpp`):

```cpp
void displaySettingsMenu() {
    // ...existing code...
    Serial.println("mppt [on/off] - Enable/disable MPPT (Maximum Power Point Tracking)");
    Serial.println("mpptthr [value] - Set MPPT threshold (0-100%, default 80%)");
    // ...existing code...
}
```

#### Step 2: Add Command Handlers

Implement the command handlers:

```cpp
void handleSettingsMenu(const String& input, XY_SKxxx* ps, XYModbusConfig& config) {
    // ...existing code...
    
    else if (input.startsWith("mppt ")) {
        String subCmd = input.substring(5);
        subCmd.trim();
        
        if (subCmd == "on") {
            if (ps->setMPPTEnable(true)) {
                Serial.println("MPPT enabled");
            } else {
                Serial.println("Failed to enable MPPT");
            }
        } else if (subCmd == "off") {
            if (ps->setMPPTEnable(false)) {
                Serial.println("MPPT disabled");
            } else {
                Serial.println("Failed to disable MPPT");
            }
        } else {
            Serial.println("Invalid option. Use 'on' or 'off'");
        }
    }
    
    // ...handle other commands...
}
```

#### Step 3: Add to Status Display

Update status display to show the new feature:

```cpp
void displayDeviceStatus(XY_SKxxx* ps) {
    // ...existing code...
    
    // Read MPPT status and threshold
    bool mpptEnabled;
    if (ps->getMPPTEnable(mpptEnabled)) {
        Serial.print("MPPT Status: ");
        Serial.println(mpptEnabled ? "ENABLED" : "DISABLED");
        
        if (mpptEnabled) {
            float mpptThreshold;
            if (ps->getMPPTThreshold(mpptThreshold)) {
                Serial.print("MPPT Threshold: ");
                Serial.print(mpptThreshold * 100, 0);
                Serial.println("%");
            }
        }
    }
    
    // ...existing code...
}
```

### 5. Testing the New Feature

1. **Compile and Flash**:
   ```
   pio run --target upload
   ```

2. **Test via Serial Monitor**:
   ```
   status       # Verify the feature appears in status output
   menu 4       # Go to settings menu
   mppt on      # Enable MPPT
   mpptthr 85   # Set threshold to 85%
   status       # Confirm changes are reflected
   ```

3. **Verify on Device**:
   Confirm the settings changed on the device's physical display.

---

**Note:** The serial monitor interface is just one example of how to expose the library features. The same library methods can be used in other interfaces such as web APIs, MQTT clients, or custom applications. The core XY-SKxxx library is designed to be interface-agnostic.

## License

[MIT License](LICENSE)
