---
source_url: https://forum.allaboutcircuits.com/threads/exploring-programming-a-xy6020l-power-supply-via-modbus.197022/
fetched: 2026-04-27
note: Content extracted via WebFetch; long quoted blocks and signatures trimmed; technical content preserved.
---

# Exploring Programming a XY6020L Power Supply via Modbus

## Thread Overview
A forum discussion on the All About Circuits Power Electronics forum where users collaborate to reverse-engineer the Modbus protocol for controlling a Chinese XY6020L variable buck power supply.

---

## Post #1 - Curt Carpenter (Nov 4, 2023)
**Thread Starter**

Curt purchased an XY6020L from AliExpress with a serial Modbus interface but lacks documentation. He seeks guidance on establishing communication, including discovering the baud rate and obtaining Modbus responses from the device.

---

## Post #2 - MisterBill2 (Nov 6, 2023)

Brief response recommending study of the Modbus protocol itself.

---

## Post #3 - Curt Carpenter (Nov 6, 2023)

Curt confirms he studied the Modbus specification but needs practical information about "pinging" the device. He notes the supply has default address 1 and likely uses 115200 baud rate.

---

## Post #4 - nsaspook (Nov 6, 2023)

Points out the user needs the device's register manual, referencing a Chinese resource (Baidu link no longer accessible).

---

## Post #5 - g-radmac (Nov 13, 2023)

**Critical Technical Contribution**

Documents experimental findings:

**Port Settings:** 115200 baud
**Protocol:** Last 2 bytes are CRC; 1st byte is slave address (0x01); 2nd byte is function code

**Register Map:**
- `0x0000` = Set voltage (read/write)
- `0x0001` = Max current
- `0x0002` = Voltage applied on output (read-only)
- `0x0012` = Output on/off control

**Functions:**
- `0x03` = Read analog output holding registers
- `0x06` = Set analog output holding registers

**Command Examples:**

Read 20 registers starting at address 0x0000:
```
01 03 00 00 00 14 45 C5
```

Response example:
```
01 03 28 01 F4 01 90 01 F4 00 00 00 00 0B DC 00 00 00 00 00 00
00 00 00 00 00 32 00 1E 01 32 22 B8 00 00 00 00 00 00 00 01
00 00 B7 D1
```

Data interpretation:
- Register 0: `0x01F4` (500) = 5.00V output
- Register 1: `0x0190` (400) = 4.00A current limit
- Register 2: `0x01F4` (500) = 5.00V actual output

**Control Commands:**
- Turn on: `01 06 00 12 00 01 E8 0F`
- Turn off: `01 06 00 12 00 00 29 CF`
- Set 5V: `01 06 00 00 01 F4 89 DD`
- Set 15V: `01 06 00 00 05 DC 8B 03`

---

## Post #7 - nsaspook (Nov 13, 2023)

Confirms the information and expresses interest in using the device for a solar energy project with Modbus interface.

---

## Post #9 - hardym2 (Dec 26, 2023)

Asks clarifying questions about:
- Whether WiFi adapter (XY-WFPOW) uses Modbus protocol
- Modbus pinout for 4-hole interface
- Voltage levels (mentions -5V concern)
- Arduino interface compatibility
- Register 6 interpretation (suspected input voltage reading at 0x0DBC = 3036 = 30.36V)

Proposes using the device as a solar charge controller for real-time adjustments.

---

## Post #10 - hardym2 (Dec 27, 2023)

Clarifies that Modbus is a protocol independent of hardware interface. Identifies the XY-WFPOW WiFi adapter as the Modbus-enabled solution, noting Android app availability and cross-references documentation sources.

---

## Post #11 - discord (Jan 7, 2024)

Questions whether the XY6020L is a buck converter or buck-boost converter, confused by conflicting marketing language.

---

## Post #12 - Hockey (Jan 7, 2024)

Clarifies the device is a **buck converter only** (steps down voltage). Maximum output voltage formula: `(input voltage + 1.1) - 2`. Example: 15V input could step down to 14.1V minimum, but stepping up requires proportionally higher input voltage.

---

## Post #13 - discord (Jan 7, 2024)

Acknowledges confusion from seller listings marketing it as "Buck Boost Converter" despite it being step-down only.

---

## Post #14 - MisterBill2 (Jan 8, 2024)

Notes that online sellers frequently make mistakes in product descriptions, attributing errors to writer carelessness or AI-generated content misinterpretation.

---

## Post #15 - jens234323 (Feb 18, 2024)

**Library Development**

Created an Arduino library for controlling the XY6020L via Arduino Pro Micro clone. Register meanings documented in header file. Control/transmit rate: ~200ms. Notes occasional command ignoring with no response.

**GitHub:** `xy6020l` repository

---

## Post #16 - stealth8020 (Mar 29, 2024)

**Documentation Discovery**

Reports that Jens's library doesn't work with ESP boards. Adapted register information for Wemos D1 Pro implementation with web-based GUI.

**Found comprehensive documentation:** XY6020L Modbus Interface PDF (via GitHub: `creepystefan/esphome-XY6020`)

**GitHub:** `xy6020_wifi_ctlr` repository for ESP8266 control application

---

## Post #17 - Curt Carpenter (Apr 3, 2024)

**Project Completion**

Completed power supply project using WEMOS D1 ESP8266 with UDP connection to PC and 20-key remote interface. Implemented control software in Free Pascal/Lazarus.

**Performance Notes:**
- Settling time after control changes: 4-15+ seconds (especially with current limiting)
- Voltage range tested: 0-35V
- Noise performance: Good (tested up to 150mA output)

Provides screenshot of completed PC interface.

---

## Key Findings Summary

The thread documents community-driven reverse engineering of an undocumented device:

1. **Baud rate:** 115200
2. **Protocol:** Standard Modbus RTU with CRC
3. **Voltage representation:** Register values in 0.01V increments
4. **Current representation:** Register values in 0.01A increments
5. **Implementation:** Multiple successful Arduino/ESP8266 implementations documented with links to working code
