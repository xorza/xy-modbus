# XY7025 / XY6020L Programmable Buck Converter — Protocol Reference

A consolidated, English-language reference for the XY-series CNC programmable
DC buck converters (XY6020L, XY7025, and the closely-related XY-SK60 /
XY-SK120 / SK120X). Compiled from a translated Chinese seller manual, a
reverse-engineered Arduino library, the device on a bench, and the firmware
in this repo.

The vendor does not publish English documentation. Treat everything below as
"community-quality": correct on the hardware tested, but unverified against
any official spec because no public official spec exists.

---

## 1. Family overview

The XY-series modules sold under "XYSEMI" / "Sinilink" / generic AliExpress
listings share the core Modbus-RTU function codes and most register addresses.
Their physical limits, fixed-point scales, optional registers, and memory-group
layouts differ by family.

This crate's high-level API is verified for XY7025. An explicit custom profile
can cover devices with the same 14-register group layout; SK-family devices use
the raw protocol layers because their groups contain a fifteenth register.

| Model     | Vin (V) | Vout (V) | Iout (A) | Pmax (W) | Notes                               |
| --------- | ------- | -------- | -------- | -------- | ----------------------------------- |
| XY6020L   | 6–65    | 0–60     | 0–20     | 1200     | Original; community docs trace here |
| XY6015    | 6–65    | 0–60     | 0–15     | ~900     | Smaller sibling                     |
| XY7025    | 12–85   | 0–70     | 0–25     | 1750     | This project's device               |
| XY-SK60   | 6–65    | 0–60     | 0–6      | 360      | LCD-screen variant                  |
| XY-SK120  | 6–65    | 0–60     | 0–10     | 600      | Buck-only                           |
| XY-SK120X | 6–36    | 0–36     | 0–10     | 360      | Buck-boost                          |

XY7025 specs (from the seller manual):

- Voltage resolution: 0.01 V; current resolution: 0.01 A
- Voltage accuracy: ±0.5% + 1 word; current accuracy: ±0.5% + 3 words
- Output ripple: 150 mVpp typical
- Conversion efficiency: ~95%
- Storage: 10 data groups (M0–M9)
- LCD: VA color screen, full-viewing-angle
- Bare board: 111 × 72 × 45 mm
- Buttons: 5 (encoder + nav)
- MPPT mode supported (solar)
- Default protection thresholds:
  - LVP (input under-volt): 10 V (range 10–95 V)
  - OVP (output over-volt): 72 V (range 0–72 V)
  - OCP (output over-current): 27 A (range 0–27 A)
  - OPP (output over-power): 1800 W (range 0–2000 W)
  - OTP (over-temperature): 95 °C (range 0–110 °C)
  - OHP (output time): off (1 min – 99 h 59 min)
  - OAH (over capacity): off (0–9999 Ah)
  - OPH (over energy): off (0–4200 kWh)

---

## 2. Communication parameters

| Parameter        | Value                                                           |
| ---------------- | --------------------------------------------------------------- |
| Protocol         | Modbus-RTU                                                      |
| Default slave ID | `0x01` (configurable via reg `0x0018`)                          |
| Default baud     | 115200                                                          |
| Frame format     | 8 data, no parity, 1 stop (`8N1`)                               |
| Physical layer   | TTL UART (3.3 V and 5 V both work in practice)                  |
| RS-485 option    | "Onboard 485" pads on some variants; same protocol              |
| Function codes   | `0x03` read holding, `0x06` write single, `0x10` write multiple |
| CRC              | Modbus standard CRC-16 (poly `0xA001`, init `0xFFFF`)           |

The raw framing and UART layers support standard Modbus address-`0` FC06/FC10
broadcast writes. Broadcasts have no response, so a successful transport call
only confirms that the frame was transmitted. The high-level `Xy` API remains
unicast-only, and XY7025 broadcast acceptance is not hardware-verified.

Connector pinout (4-pin Molex/JST on the rear of the module):

```
[ VCC ][ TX ][ RX ][ GND ]
   |     |     |      |
   5V    DIN   DOUT   0V
```

- `VCC` (5 V out) — powers an isolated RS-485 adapter; **do not** wire to MCU 3V3
- `TX` — **module's serial input** → connect to host TX
- `RX` — **module's serial output** → connect to host RX
- `GND` — common ground

> **Pin naming gotcha.** `TX` and `RX` are labelled from the _module's_
> perspective, not the host's. Host TX → module TX (which is the module's
> input). No crossover.

### Timing requirements (empirical)

These are not in the seller manual — they come from this project's bench
testing of the XY7025 and the bundled transport (`src/uart/mod.rs`):

| Constraint           | Value            | Notes                                                                                                                                                                                        |
| -------------------- | ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Min inter-frame gap  | ~50 ms           | Tighter and the device drops back-to-back frames; confirmed by [Jens's header](https://github.com/xorza/xy-modbus/blob/master/docs-archive/jens3382-xy6020l.h#L148)                                                                         |
| Response timeout     | ~500 ms          | Worst case observed on XY7025; 200 ms is unreliable. [Jens's XY6020L library](https://github.com/xorza/xy-modbus/blob/master/docs-archive/jens3382-xy6020l.cpp) runs tighter (~40 ms) — XY7025 firmware appears to be the slower of the two |
| Quiet-bus acquisition | ≤500 ms default | The bundled transport tries at most ten 50 ms quiet intervals, then returns `RtuError::BusBusy` without transmitting                                                                         |
| Post-write quiet gap | ~10 ms           | Required before a follow-up read of the same reg                                                                                                                                             |
| Cold-boot UART ready | ~1–2 s after Vin | Slower without USB-CDC enumeration delay to mask                                                                                                                                             |

The Arduino lib's note that "tx period < 50 ms → no answers" matches what
we see on the XY7025.

---

## 3. Holding-register map (function code `0x03` / `0x06` / `0x10`)

All registers are 16-bit, big-endian on the wire (Modbus standard). The
"Scale" column is the divisor to apply to the raw integer to get the
physical value (so `1440` with scale `100` = `14.40 V`).

> **Model-specific scales — important.** The scale columns below are for
> XY6020L and XY7025. The SK family uses **higher-resolution scales** for
> current and power (and adds extra registers — see §3.6). Per the csvke
> [SK120 register PDF](https://github.com/xorza/xy-modbus/blob/master/docs-archive/csvke-XY-SK120-Modbus_Address.pdf):
>
> | Register         | XY6020L / XY7025 scale | SK120 / SK60 / SK120X scale |
> | ---------------- | ---------------------- | --------------------------- |
> | I-SET (`0x0001`) | 100 (10 mA)            | **1000 (1 mA)**             |
> | IOUT (`0x0003`)  | 100                    | **1000**                    |
> | POWER (`0x0004`) | 10 (100 mW)            | **100 (10 mW)**             |
> | S-OCP (`0x0054`) | 100                    | **1000**                    |
> | S-OPP (`0x0055`) | 1 (W)                  | **10 (0.1 W)**              |
>
> Cross-check `MODEL` (`0x0016`) before assuming a scale: `0x6100`-class
> firmware is XY6020L/XY7025 (this crate's target); SK-series firmware
> reports a different model code.

### 3.1 Status & runtime control (`0x0000 – 0x001E`)

| Addr     | Name        | Description                                                                                       | Scale | Unit  | R/W |
| -------- | ----------- | ------------------------------------------------------------------------------------------------- | ----- | ----- | --- |
| `0x0000` | V-SET       | Output voltage setpoint                                                                           | 100   | V     | R/W |
| `0x0001` | I-SET       | Output current limit setpoint                                                                     | 100   | A     | R/W |
| `0x0002` | VOUT        | Measured output voltage                                                                           | 100   | V     | R   |
| `0x0003` | IOUT        | Measured output current                                                                           | 100   | A     | R   |
| `0x0004` | POWER       | Measured output power                                                                             | 10    | W     | R   |
| `0x0005` | UIN         | Measured input voltage                                                                            | 100   | V     | R   |
| `0x0006` | AH-LOW      | Cumulative output charge, low word                                                                | 1000  | Ah    | R   |
| `0x0007` | AH-HIGH     | Cumulative output charge, high word                                                               | —     | Ah    | R   |
| `0x0008` | WH-LOW      | Cumulative output energy, low word                                                                | 1000  | Wh    | R   |
| `0x0009` | WH-HIGH     | Cumulative output energy, high word                                                               | —     | Wh    | R   |
| `0x000A` | OUT_H       | Output-on time, hours                                                                             | 1     | h     | R   |
| `0x000B` | OUT_M       | Output-on time, minutes                                                                           | 1     | min   | R   |
| `0x000C` | OUT_S       | Output-on time, seconds                                                                           | 1     | s     | R   |
| `0x000D` | T_IN        | Internal temperature                                                                              | 10    | °C/°F | R   |
| `0x000E` | T_EX        | External-probe temperature                                                                        | 10    | °C/°F | R   |
| `0x000F` | LOCK        | Front-panel key lock (0=unlocked, 1=locked)                                                       | —     | —     | R/W |
| `0x0010` | PROTECT     | Latched protection cause (see §4)                                                                 | —     | —     | R/W |
| `0x0011` | CVCC        | Regulation mode (0=CV, 1=CC)                                                                      | —     | —     | R   |
| `0x0012` | ONOFF       | Output enable (0=off, 1=on)                                                                       | —     | —     | R/W |
| `0x0013` | F-C         | Temperature unit (0=°C, 1=°F)                                                                     | —     | —     | R/W |
| `0x0014` | B-LED       | Backlight brightness (1–5; firmware floors 0 → 1)                                                 | —     | —     | R/W |
| `0x0015` | SLEEP       | Off-screen timeout (firmware caps at 9 min)                                                       | 1     | min   | R/W |
| `0x0016` | MODEL       | Product number (e.g. `0x6100`)                                                                    | —     | —     | R   |
| `0x0017` | VERSION     | Firmware version (e.g. `0x0071`)                                                                  | —     | —     | R   |
| `0x0018` | SLAVE-ADD   | Modbus slave address; takes effect after device reset                                             | —     | —     | R/W |
| `0x0019` | BAUDRATE_L  | Baud-rate code (see §3.6)                                                                         | —     | —     | R/W |
| `0x001A` | T-IN-OFFSET | Internal-temp calibration offset (XY7025: writes silently ignored over Modbus — front-panel only) | 10    | °C/°F | R   |
| `0x001B` | T-EX-OFFSET | External-temp calibration offset (XY7025: writes silently ignored over Modbus — front-panel only) | 10    | °C/°F | R   |
| `0x001C` | BUZZER      | Buzzer enable (often unimplemented)                                                               | —     | —     | R/W |
| `0x001D` | EXTRACT-M   | Recall memory group (write 0–9)                                                                   | —     | —     | R/W |
| `0x001E` | DEVICE      | Device status — unreliable on some FW                                                             | —     | —     | R/W |

`read_totals` fetches `0x0006`–`0x000C` in one FC03 transaction, but neither
the Modbus specification nor the device documentation guarantees that the
LOW/HIGH counter pairs are snapshotted atomically. A read concurrent with a
low-word rollover can therefore tear; accounting-sensitive applications should
verify the high words before and after the block or retry around rollovers.

### 3.2 SK-family extras (`0x001F – 0x0023`)

Documented in the [SK120 register PDF](https://github.com/xorza/xy-modbus/blob/master/docs-archive/csvke-XY-SK120-Modbus_Address.pdf)
p.1 and exercised by the [archived csvke README](https://github.com/xorza/xy-modbus/blob/master/docs-archive/csvke-README.md#L148-L153). **Not present** on XY6020L per
the tinkering4fun PDF (which ends at `0x001E`). The XY7025 marketing
material advertises both MPPT and constant-power modes, so these
registers are likely present on XY7025 too — but unverified at the
register level. Use with caution on non-SK hardware.

| Addr     | Name    | Description                                      | R/W |
| -------- | ------- | ------------------------------------------------ | --- |
| `0x001F` | MPPT-SW | MPPT (solar maximum-power-point tracking) enable | R/W |
| `0x0020` | MPPT-K  | MPPT max-power-point coefficient                 | R/W |
| `0x0021` | BatFul  | Battery-full cutoff current                      | R/W |
| `0x0022` | CW-SW   | Constant-power mode enable                       | R/W |
| `0x0023` | CW      | Constant-power setpoint                          | R/W |

### 3.3 WiFi pairing (`0x0030 – 0x0034`)

Only populated when a SiniLink XY-WFPOW (ESP8285) WiFi board is attached.

| Addr     | Name        | Description                                    |
| -------- | ----------- | ---------------------------------------------- |
| `0x0030` | MASTER      | Host type (`0x3B3A` = WiFi)                    |
| `0x0031` | WIFI-CONFIG | Pairing mode: 0 invalid / 1 touch / 2 AP       |
| `0x0032` | WIFI-STATUS | 0 none / 1 router / 2 server / 3 touch / 4 AP  |
| `0x0033` | IPV4-H      | High 16 bits of IPv4 (e.g. `0xC0A8` = 192.168) |
| `0x0034` | IPV4-L      | Low 16 bits of IPv4 (e.g. `0x0108` = .1.8)     |

### 3.4 Active parameter set M0 (`0x0050 – 0x005D`)

Memory group M0 is the **live operating set**. Writing here takes effect
immediately. Registers `0x0050`/`0x0051` are aliases of `0x0000`/`0x0001`
— the device mirrors them. The other M0 registers have **no aliases** in
the `0x000x` block, so this is where you program protection thresholds
and the power-on-output behavior.

| Addr     | Name    | Description                                                           | Scale | Unit  | R/W |
| -------- | ------- | --------------------------------------------------------------------- | ----- | ----- | --- |
| `0x0050` | V-SET   | Mirror of `0x0000`                                                    | 100   | V     | R/W |
| `0x0051` | I-SET   | Mirror of `0x0001`                                                    | 100   | A     | R/W |
| `0x0052` | S-LVP   | Input low-voltage protection threshold                                | 100   | V     | R/W |
| `0x0053` | S-OVP   | Output over-voltage protection threshold                              | 100   | V     | R/W |
| `0x0054` | S-OCP   | Output over-current protection threshold                              | 100   | A     | R/W |
| `0x0055` | S-OPP   | Output over-power protection threshold                                | 1     | W     | R/W |
| `0x0056` | S-OHP_H | Max output time, hours                                                | 1     | h     | R/W |
| `0x0057` | S-OHP_M | Max output time, minutes                                              | 1     | min   | R/W |
| `0x0058` | S-OAH_L | Max output charge, low 16 bits                                        | 1000  | Ah    | R/W |
| `0x0059` | S-OAH_H | Max output charge, high 16 bits                                       | —     | Ah    | R/W |
| `0x005A` | S-OWH_L | Max output energy, low 16 bits (10 mWh units)                         | 100   | Wh    | R/W |
| `0x005B` | S-OWH_H | Max output energy, high 16 bits (10 mWh units)                        | —     | Wh    | R/W |
| `0x005C` | S-OTP   | Over-temperature protection (raw value = displayed °, see note below) | 1     | °C/°F | R/W |
| `0x005D` | S-INI   | Power-on output state (0=off, 1=on, persists in EEPROM)               | —     | —     | R/W |

> **S-OTP scale (resolved empirically on XY7025).** Storage is unscaled:
> the raw register value equals the displayed degrees in the unit
> selected by `F-C` (raw 95 with unit=°F is 95 °F; raw 110 with unit=°C
> is 110 °C). This was verified on real hardware by writing every value
> in `[10, 1100]` via single-register writes — all round-tripped
> identically in both °C and °F mode. The tinkering4fun PDF's "scale 10"
> entry is wrong for this firmware; the csvke / Jens-Gleissberg "scale 1"
> reading is correct. Note: `write_multiple` (group writes) routes
> through firmware unit conversion which clamps to 110 °C / 230 °F and
> introduces ±1° rounding — single-register writes do neither.
> The high-level `write_group` API converts its unit-tagged threshold to the
> active `F-C` unit, rejects values above that limit, and returns the stored
> group readback so callers observe any firmware rounding.
> Group reads and writes use multiple Modbus transactions. Do not change `F-C`
> concurrently through another controller or the front panel while either
> operation is in progress.

### 3.5 Memory groups M0–M9 (`0x0050 + N × 0x0010`)

The device stores 10 preset groups, each 14 registers wide on
XY6020L/XY7025:

```
M_N base address = 0x0050 + (N × 0x0010)
```

| Group | Base     | Notes                                        |
| ----- | -------- | -------------------------------------------- |
| M0    | `0x0050` | Live operating parameters (writes apply now) |
| M1    | `0x0060` | Quick-recall slot 1 (front-panel button)     |
| M2    | `0x0070` | Quick-recall slot 2 (front-panel button)     |
| M3    | `0x0080` | General preset                               |
| M4    | `0x0090` | General preset                               |
| M5    | `0x00A0` | General preset                               |
| M6    | `0x00B0` | General preset                               |
| M7    | `0x00C0` | General preset                               |
| M8    | `0x00D0` | General preset                               |
| M9    | `0x00E0` | General preset                               |

Inside each group the layout matches §3.4 (V-SET, I-SET, S-LVP, S-OVP,
S-OCP, S-OPP, S-OHP_H, S-OHP_M, S-OAH_L, S-OAH_H, S-OWH_L, S-OWH_H,
S-OTP, S-INI — 14 registers).

> **SK family is 15 registers wide.** The csvke SK120 register PDF (p.2)
> adds an extra `S-ETP` (external over-temperature protection) at offset
> `+14` (`0x005E` on M0). On SK120/SK60/SK120X, plan for 15-register
> groups and a stride that still places M1 at `0x0060`. This crate's high-level
> group API uses the 14-register XY7025 layout and does not expose SK groups.

**Recall semantics.** Writing `1`–`9` to `EXTRACT-M` (`0x001D`) copies
that group's contents into M0; the change takes effect immediately.
Writing `0` is a no-op — M0 is already current.

**Programming a non-active preset** (M1–M9) updates EEPROM but does
**not** change the live operating parameters until that preset is
recalled.

### 3.6 Baud-rate codes (`0x0019` BAUDRATE_L)

The seller manual documents `6 == 115200` only. No primary source in
this archive maps the other codes — [Jens Gleissberg's library](https://github.com/xorza/xy-modbus/blob/master/docs-archive/jens3382-xy6020l.h#L230-L231)
explicitly notes "no read option …
@todo: provide enum", and the csvke files contain no baud-code
mapping either. The mapping below is **community speculation** and was
removed from the rewritten upstream sources; included here only because
some forks repeat it:

| Code | Claimed baud (unverified) |
| ---- | ------------------------- |
| 0    | 9600                      |
| 1    | 14400                     |
| 2    | 19200                     |
| 3    | 38400                     |
| 4    | 56000                     |
| 5    | 57600                     |
| 6    | 115200 _(documented)_     |
| 7    | 2400                      |
| 8    | 4800                      |

Treat anything other than `6` as unverified — read it back, observe the
device after a reset, or stick to the factory default. Baud changes take
effect after the device power-cycles.

---

## 4. Protection (`PROTECT` register `0x0010`)

Read codes:

| Value | Cause  | Meaning                              |
| ----- | ------ | ------------------------------------ |
| 0     | Normal | Operating normally                   |
| 1     | OVP    | Output over-voltage                  |
| 2     | OCP    | Output over-current                  |
| 3     | OPP    | Output over-power                    |
| 4     | LVP    | Input under-voltage                  |
| 5     | OAH    | Cumulative charge limit reached      |
| 6     | OHP    | Output time limit reached            |
| 7     | OTP    | Over-temperature                     |
| 8     | OEP    | Internal power-stage/no-output protection; exact trigger unverified |
| 9     | OWH    | Cumulative energy limit reached (Wh) |
| 10    | ICP    | Input over-current / inrush          |

**Behavior on trip.** The output disables, the front-panel backlight
blinks, and the LCD shows the trip code. Writing `0` to `PROTECT`
(`0x0010`) clears the latched cause and stops the blink — but does
**not** re-enable the output; you must also write `1` to `ONOFF`
(`0x0012`).

The [SK-family protocol table](https://github.com/xorza/xy-modbus/blob/master/docs-archive/csvke-XY-SK120X.pdf)
describes `OEP` as “no-output protection,” and a
[related XY power-supply manual](https://supereyes.ru/img/instructions/XY-SEP4_manual.pdf)
uses it when the converter chip's own protection fires. It is not another
charge or energy limit; the precise XY7025 trigger set remains unverified.

**OVP-on-V-SET-write quirk.** If you write a `V-SET` higher than the
current `S-OVP`, the device latches OVP immediately, even if the output
is off. Always program `S-OVP` (`0x0053`) before raising `V-SET`.
Documented in the [original seller manual](https://github.com/xorza/xy-modbus/blob/master/docs-archive/tinkering4fun-XY6020L-Modbus-Interface.pdf)
p.6, Note 3: "OVP is triggered when a programming request for a
higher voltage is made (e.g. write to register V-SET 0000H)".

---

## 5. CRC-16 (Modbus)

Standard Modbus CRC. Polynomial `0xA001` (reflected `0x8005`), init
`0xFFFF`, no final XOR, byte-reflected in/out.

```
crc = 0xFFFF
for byte b in frame:
    crc ^= b
    for _ in 0..8:
        if crc & 1: crc = (crc >> 1) ^ 0xA001
        else:        crc = crc >> 1
# Append CRC low byte first, then high byte (little-endian on wire).
```

---

## 6. Wire-level examples

### Read VOUT + IOUT (registers `0x0002` and `0x0003`)

Request:

`01 03 00 02 00 02 65 CB`

| Offset | Bytes   | Meaning                                      |
|--------|---------|----------------------------------------------|
| 0      | `01`    | Slave `0x01`                                 |
| 1      | `03`    | Read Holding Registers                       |
| 2–3    | `00 02` | Starting register `0x0002` (`VOUT`)          |
| 4–5    | `00 02` | Quantity: 2 registers                        |
| 6–7    | `65 CB` | CRC `0xCB65`, transmitted low byte first     |

Response (Vout=5.00 V, Iout=0.00 A):

`01 03 04 01 F4 00 00 BA 3D`

| Offset | Bytes   | Meaning                                      |
|--------|---------|----------------------------------------------|
| 0      | `01`    | Slave `0x01`                                 |
| 1      | `03`    | Function-code echo                           |
| 2      | `04`    | Payload byte count: 4                        |
| 3–4    | `01 F4` | Register `0x0002`: `VOUT = 500`              |
| 5–6    | `00 00` | Register `0x0003`: `IOUT = 0`                |
| 7–8    | `BA 3D` | CRC `0x3DBA`, transmitted low byte first     |

`0x01F4` = 500 → 5.00 V; `0x0000` = 0 → 0.00 A.

### Set V-SET to 14.40 V (write single, FC `0x06`)

Encoded value `1440` = `0x05A0`:

`01 06 00 00 05 A0 09 7E`

| Offset | Bytes   | Meaning                                      |
|--------|---------|----------------------------------------------|
| 0      | `01`    | Slave `0x01`                                 |
| 1      | `06`    | Write Single Holding Register                |
| 2–3    | `00 00` | Register `0x0000` (`V-SET`)                  |
| 4–5    | `05 A0` | Value `1440`                                 |
| 6–7    | `09 7E` | CRC `0x7E09`, transmitted low byte first     |

Echo response is identical to the request (FC `0x06` reflects).

### Program protection block (write multiple, FC `0x10`)

Set `S-LVP=10.00 V`, `S-OVP=15.00 V`, `S-OCP=12.50 A` in one frame
(addresses `0x0052`–`0x0054`):

`01 10 00 52 00 03 06 03 E8 05 DC 04 E2 67 90`

| Offset | Bytes   | Meaning                                      |
|--------|---------|----------------------------------------------|
| 0      | `01`    | Slave `0x01`                                 |
| 1      | `10`    | Write Multiple Holding Registers             |
| 2–3    | `00 52` | Starting register `0x0052` (`S-LVP`)         |
| 4–5    | `00 03` | Quantity: 3 registers                        |
| 6      | `06`    | Payload byte count: 6                        |
| 7–8    | `03 E8` | `S-LVP = 1000` (`10.00 V`)                  |
| 9–10   | `05 DC` | `S-OVP = 1500` (`15.00 V`)                  |
| 11–12  | `04 E2` | `S-OCP = 1250` (`12.50 A`)                  |
| 13–14  | `67 90` | CRC `0x9067`, transmitted low byte first     |

---

## 7. Bring-up checklist

A safe boot sequence for charging applications:

1. `set_output(false)` — write `0` to `0x0012` before anything else.
2. `clear_protection_status` — write `0` to `0x0010` (wipes any
   stale latched cause from a previous session).
3. `set_power_on_output(false)` — write `0` to `S-INI` (`0x005D`) so
   the buck always boots disabled even after a brown-out / MCU
   crash.
4. `set_protection(LVP, OVP, OCP)` — program `0x0052`–`0x0054`
   **before** raising `V-SET` (otherwise OVP latches on write).
5. `set_voltage` (`0x0000`) and `set_current_limit` (`0x0001`).
6. **Read everything back** with `read_status()` and `read_group(0)`; verify
   the live snapshot, setpoints, and safety limits. This catches dropped
   writes, scale-divider mismatches, and wrong-slave wiring.
7. Check `status.output_on` from `read_status()` before handing off to the
   supervisor. If it is `true`, the disable in step 1 did not stick—refuse to
   enable.
8. Only now: `set_output(true)` if the supervisor decides it's
   safe.

For polling, use `read_status()` to fetch `0x0000`–`0x0012` in one transaction,
including setpoints, live measurements, protection status, regulation mode,
and output state.

---

## 8. Known quirks and gotchas

- **`SLAVE-ADD` change requires reset** — the new address only
  becomes active after the device power-cycles. The Arduino lib
  documents this; we have not verified on the XY7025.
- **`BAUDRATE_L` change requires reset** — same caveat as
  slave address.
- **`DEVICE` register `0x001E`** is documented but flaky on the
  XY6020L tested by tinkering4fun. Don't depend on it.
- **`BUZZER` register `0x001C`** appears unimplemented on at
  least some firmware revisions.
- **Min inter-frame gap ~50 ms** — back-to-back writes inside this
  window go unanswered. `src/uart/mod.rs` enforces this before every
  request; tighten with care.
- **`AH-HIGH` / `WH-HIGH` testing** — the original community docs
  flag these high words as untested, and the firmware's cross-word snapshot
  behavior is undocumented. Don't assume an FC03 response cannot tear at a
  low-word rollover without verifying on your hardware.
- **Temperature in F vs C** — `T_IN`/`T_EX` units are governed by
  `F-C` (`0x0013`). Set this to `0` (Celsius) at boot if you want
  predictable readings.
- **Protection-status read while output is on** — when the buck is
  actively sourcing, `PROTECT` is necessarily `0`. Only worth
  reading when `ONOFF` is `0` and you want to know _why_.

---

## 9. References

The information in this document was compiled from:

1. [tinkering4fun/XY6020L-Modbus](https://github.com/tinkering4fun/XY6020L-Modbus) —
   English translation of the original Chinese seller manual
   (`doc/XY6020L-Modbus-Interface.pdf`, April 2023, public domain).
   Primary source for the register table, protection codes, and
   memory-group layout.
2. [Jens3382/xy6020l](https://github.com/Jens3382/xy6020l) — Arduino
   library by Jens Gleissberg (LGPL-3.0+). Source of the
   `HREG_IDX_*` constants and the 50 ms tx-period observation.
   Credits user `g-radmac` for the original UART-protocol reverse
   engineering.
3. [allaboutcircuits.com forum thread](https://forum.allaboutcircuits.com/threads/exploring-programming-a-xy6020l-power-supply-via-modbus.197022/) —
   Original community discovery of the XY6020L Modbus protocol.
4. [csvke/XY-SK120-Modbus-RTU-TTL](https://github.com/csvke/XY-SK120-Modbus-RTU-TTL) —
   C++ Arduino library for the XY-SK120 confirming the protocol
   carries across the SK series.
5. XY7025 seller manual at [manuals.plus](https://manuals.plus/ae/1005008036046439) —
   physical specs, accuracy, protection ranges. The PDF does
   **not** include a Modbus register map.
6. [`src/uart/mod.rs`](src/uart/mod.rs) and
   [`src/device/mod.rs`](src/device/mod.rs) in this repository — the
   bundled transport timing, high-level register operations, and the
   `S-INI=0` rationale for charging applications.
