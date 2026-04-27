---
source_url: https://manuals.plus/ae/1005008036046439
fetched: 2026-04-27
note: Content extracted via WebFetch; preserves seller manual text including specs and protections.
---

# XY7025 CNC DC Adjustable Buck Power Supply Module - User Manual

## 1. Introduction

The diymore XY7025 is "a high-performance CNC DC adjustable buck power supply module designed for precise voltage and current regulation." It supports constant voltage (CV), constant current (CC), constant power (CW) modes, and includes MPPT solar charging capabilities.

## 2. Key Features

- **Intuitive Interface:** Large LCD screen (1.8+ inches) with silicone buttons and indicator lights
- **Robust Construction:** Flame-retardant shell, double-layer buckle design, double-layer circuit board
- **Advanced Communication:** Modbus protocol support, dual serial ports (TTL and 485 communication)
- **MPPT Solar Charging:** "Integrated Maximum Power Point Tracking function for efficient solar charging"
- **High-Quality Components:** PCB welded terminals (M3), iron-silicon-aluminum inductors, temperature-controlled fan
- **Precise Control:** External clock crystal oscillator, 48-pin MCU (64M frequency)
- **Versatile Connectivity:** Compatible with WiFi (XY-K485X), infrared, and 433M wireless modules

## 3. Specifications

| Parameter | Value |
|-----------|-------|
| Model Number | XY7025 |
| Input Voltage | DC 12-85V |
| Output Voltage | 0-70V |
| Output Current | 0-25A |
| Power Output | 1750W |
| Voltage Resolution | 0.01V |
| Current Resolution | 0.01A |
| Voltage Accuracy | ±0.5%+1 word |
| Current Accuracy | ±0.5%+3 words |
| Conversion Efficiency | ~95% |
| Storage Data Groups | 10 groups |
| MPPT Function | Yes |
| Output Ripple (Typical) | VPP-150mv |
| Bare Board Size | 111 x 72 x 45mm |
| Screen Instrument Size | 86 x 45 x 50mm |
| Number of Buttons | 5 |

## 4. Safety Precautions

- "DO NOT connect the input to AC power or 220V utility power"
- Maximum input voltage must not exceed 90V (85V recommended)
- "DO NOT reverse the positive and negative input terminals"
- Output lacks reverse current protection; use a series diode when charging batteries
- "DO NOT reverse the positive and negative output terminals when connecting a battery"
- "This product is a step-down power supply. The maximum output voltage is approximately 94.5% of the input voltage"
- Battery charging requires professional expertise to prevent fire/explosion

## 5. Setup and Connections

### 5.1 Bare Board Configuration

When purchasing the bare device, "switches, potentiometers, and dial switches are pre-welded." Screen meter sets lack these pre-welded components.

### 5.2 Wiring Terminals

- **IN+, IN-:** Input power (DC 12-85V)
- **OUT+, OUT-:** Output power (0-70V, 0-25A)
- **Power Switch:** Hard ON/OFF switch
- **NTC Temperature Sensor:** 10k NTC 3950B external sensor interface
- **Serial Communication (TTL):** Device communication
- **485 Communication (AB):** Long-distance control interface
- **MPPT ON/OFF:** Hardware switch with indicator light

### 5.3 Multi-Module Communication (485 Bus)

"Multiple modules can be connected via an RS485 bus for synchronized operation." Independent dual serial ports (TTL and 485) operate simultaneously.

### 5.4 Optional Accessories

- XY-K485X Module (485 bus control)
- WiFi Module (APP/web-based networking)
- Infrared Receiver Module (remote control)
- 433M Wireless Remote Control Module (multi-device synchronization)

## 6. Operating Instructions

### 6.1 Power On/Off

- **Power On:** Short press the ON/OFF button
- **Power Off:** Long press ON/OFF for 5 seconds

### 6.2 Setting Voltage (CV) and Current (CC)

1. Short press V-SET button on main UI; "LCD will display 'SET' in the lower row, and 'CV' will flash"
2. Short press SW button or rotate encoder to shift voltage setting position
3. Rotate encoder to adjust value
4. Short press V-SET to exit and save
5. Follow same procedure for current (I-SET button)

**Quick Setting:** In system parameter settings, set FET to CV or CC. Rotate encoder on main UI to enter setting mode, then rotate to adjust quickly.

### 6.3 Display Modes

- **Input/Output Voltage:** Short press SW button to toggle between IN and ON displays
- **Parameter Cycling:** Short press encoder button on main UI to cycle through power (W), capacity (Ah), energy (Wh), time (h), and temperature (°C)

### 6.4 Key Lock Function

- Long press encoder button for 2 seconds to lock voltage/current settings; lock symbol appears
- Long press encoder button again for 2 seconds to unlock

### 6.5 Data Group Function (Cd0-Cd9)

Module supports 10 data groups for storing/recalling settings:

1. Long press V-SET button on main UI to access data group interface
2. Rotate encoder to cycle through desired data group (Cd0-Cd9)
3. Short press V-SET or I-SET to switch between CV and CC settings
4. Long press V-SET/SW or short press encoder button to select group

### 6.6 Constant Power (CW) Function

**Without Constant Power:** Module switches automatically between CV and CC modes based on load.
- If load current < set constant current: CV mode (voltage = set value, current adaptive)
- If load current > set constant current: CC mode (current = set value, voltage adaptive)

**With Constant Power:** "When enabled, the constant current value defaults to maximum, and the constant voltage value (CV) serves as the initial voltage." Module calculates equivalent resistance (R=U/I) and maintains constant power via algorithm.

**Enabling Constant Power:**
1. Long press SW on main UI to enter system settings
2. Short press I-SET/V-SET to switch to constant power option
3. Press ON to enable, OFF to disable
4. Short press I-SET on main interface to modify constant power value

### 6.7 Bare Board Potentiometer Mode

For bare board without screen meter, "output voltage (constant voltage) can be controlled via the potentiometer." Constant current set via dip switch offering 16 gear settings, eliminating need for multimeter current measurement.

## 7. Protection Mechanisms

| Protection Type | Description | Adjustable Range / Default |
|-----------------|-------------|---------------------------|
| Input Undervoltage (LVP) | Protects against low input voltage | 10-95V adjustable, default 10V |
| Output Overvoltage (OVP) | Protects against excessive output voltage | 0-72V adjustable, default 72V |
| Output Overcurrent (OCP) | Protects against excessive output current | 0-27A adjustable, default 27A |
| Output Overpower (OPP) | Protects against excessive output power | 0-2000W adjustable, default 1800W |
| Over-temperature (OTP) | Protects against overheating | 0-110°C adjustable, default 95°C |
| Timeout (OHP) | Shuts off output after set time | 1 min - 99h 59min, default off |
| Over Capacity (OAH) | Shuts off output after set charge capacity | 0-9999Ah adjustable, default off |
| Super Energy (OPH) | Shuts off output after set energy output | 0-4200KWH adjustable, default off |

## 8. Maintenance

- Keep module clean and free from dust/moisture
- Ensure proper ventilation during high-power operation (intelligent temperature-controlled fan included)
- Regularly check wiring for tightness and wear
- Refer to official documentation for firmware upgrades
- For bare board: ensure dip switches correctly set for desired constant current

## 9. Troubleshooting

**No Power/Display:**
- Verify input voltage is within 12-85V range
- Confirm power switch is ON
- Check for reverse polarity at input

**Incorrect Output Voltage/Current:**
- Recheck V-SET and I-SET button settings
- Verify bare board dip switch settings if applicable
- Ensure load is correctly connected without short circuits

**Module Shuts Down Unexpectedly:**
- Check active protection mechanisms (LVP, OVP, OCP, OPP, OTP, OHP, OAH, OPH) on display
- Verify adequate cooling and fan operation
- Confirm input voltage stability

**Communication Issues:**
- Check serial port connections (TTL/485)
- Verify correct baud rates and protocols in software
- Consult multi-serial port PC software documentation

**MPPT Not Functioning:**
- Ensure MPPT hardware switch is ON
- Verify solar panel connection and adequate sunlight
- Check external temperature probe connection if used

## 10. User Tips

- Use encoder for fine-tuning after selecting digit with SW button for precise adjustments
- Utilize 10 data groups to save frequently used presets for quick recall
- When charging batteries, verify polarity and ensure output voltage is slightly higher than battery voltage, or use external diode
- "The multi-serial port PC software offers advanced control and monitoring capabilities" for complex setups

## 11. Warranty and Support

Contact manufacturer or point of purchase for warranty information and technical support. Manufacturer provides complementary Multi-Serial Port Host Software supporting "up to 247 devices on a single serial port" with batch control and administrative accounts. Software updates regularly released.
