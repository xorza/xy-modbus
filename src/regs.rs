//! Holding-register addresses for the XY-series buck converters.
//!
//! All registers are 16-bit. Where a value is fixed-point, the *scale*
//! is the divisor to apply to the raw integer to obtain the physical
//! value (so a raw `1440` at scale `100` is `14.40`).

// This module is the canonical address surface for the whole register
// map. Some entries aren't referenced by the high-level API yet (rarely
// used registers, group offsets, individual M0 protection fields) but
// are kept so the map stays complete in one place.
#![allow(dead_code)]

/// Default Modbus slave address. Reconfigurable via [`REG_SLAVE_ADDR`];
/// the new value only takes effect after the device resets.
pub const DEFAULT_SLAVE: u8 = 0x01;

// ─── Status & runtime control (0x0000 – 0x001E) ──────────────────────────────

/// V-SET — output voltage setpoint. Scale 100 (V).
pub const REG_V_SET: u16 = 0x0000;
/// I-SET — output current limit setpoint. Scale 100 (A).
pub const REG_I_SET: u16 = 0x0001;
/// V-OUT — measured output voltage. Scale 100 (V).
pub const REG_V_OUT: u16 = 0x0002;
/// I-OUT — measured output current. Scale 100 (A).
pub const REG_I_OUT: u16 = 0x0003;
/// P-OUT — measured output power. Scale 10 (W).
pub const REG_P_OUT: u16 = 0x0004;
/// V-IN — measured input voltage. Scale 100 (V).
pub const REG_V_IN: u16 = 0x0005;
/// AH-LOW — cumulative output charge, low word. Scale 1000 (Ah).
pub const REG_AH_LOW: u16 = 0x0006;
/// AH-HIGH — cumulative output charge, high word. Untested in
/// community docs; treat the 32-bit composition as best-effort.
pub const REG_AH_HIGH: u16 = 0x0007;
/// WH-LOW — cumulative output energy, low word. Scale 1000 (Wh).
pub const REG_WH_LOW: u16 = 0x0008;
/// WH-HIGH — cumulative output energy, high word.
pub const REG_WH_HIGH: u16 = 0x0009;
/// OUT-H — output-on time, hours.
pub const REG_OUT_H: u16 = 0x000A;
/// OUT-M — output-on time, minutes.
pub const REG_OUT_M: u16 = 0x000B;
/// OUT-S — output-on time, seconds.
pub const REG_OUT_S: u16 = 0x000C;
/// T-IN — internal temperature. Scale 10 (°C/°F per [`REG_TEMP_UNIT`]).
pub const REG_T_IN: u16 = 0x000D;
/// T-EX — external probe temperature. Scale 10 (°C/°F).
pub const REG_T_EX: u16 = 0x000E;
/// LOCK — front-panel key lock (0 unlocked, 1 locked).
pub const REG_LOCK: u16 = 0x000F;
/// PROTECT — latched protection cause. Write 0 to clear.
pub const REG_PROTECT: u16 = 0x0010;
/// CVCC — regulation mode (0 CV, 1 CC).
pub const REG_CVCC: u16 = 0x0011;
/// ONOFF — output enable (0 off, 1 on).
pub const REG_OUTPUT_EN: u16 = 0x0012;
/// F-C — temperature unit (0 °C, 1 °F).
pub const REG_TEMP_UNIT: u16 = 0x0013;
/// B-LED — backlight brightness (0–5).
pub const REG_BACKLIGHT: u16 = 0x0014;
/// SLEEP — off-screen timeout in minutes.
pub const REG_SLEEP: u16 = 0x0015;
/// MODEL — product number (e.g. `0x6500` on XY7025).
pub const REG_MODEL: u16 = 0x0016;
/// VERSION — firmware version (e.g. `0x0071`).
pub const REG_VERSION: u16 = 0x0017;
/// SLAVE-ADD — Modbus slave address; takes effect after device reset.
pub const REG_SLAVE_ADDR: u16 = 0x0018;
/// BAUDRATE_L — baud-rate code (see [`crate::types::BaudRate`]).
pub const REG_BAUD_CODE: u16 = 0x0019;
/// T-IN-OFFSET — internal-temp calibration offset. Scale 10.
pub const REG_T_IN_OFFSET: u16 = 0x001A;
/// T-EX-OFFSET — external-temp calibration offset. Scale 10.
pub const REG_T_EX_OFFSET: u16 = 0x001B;
/// BUZZER — buzzer enable. Often unimplemented in firmware.
pub const REG_BUZZER: u16 = 0x001C;
/// EXTRACT-M — recall a memory group into M0 (write 0–9).
pub const REG_EXTRACT_M: u16 = 0x001D;
/// DEVICE — device status. Documented but unreliable on some firmware.
pub const REG_DEVICE: u16 = 0x001E;

// ─── WiFi pairing (0x0030 – 0x0034) — pending ────────────────────────────────
//
// Only populated when a SiniLink XY-WFPOW (ESP8285) board is attached.
// Register addresses are listed here for reference; the high-level API
// in `device.rs` does not yet expose them — see the README for the
// register layout if you want to drive these directly via
// `ModbusTransport`.
//
//   0x0030  WiFi MASTER       host type (0x3B3A = WiFi)
//   0x0031  WiFi CONFIG       pairing mode (0 invalid / 1 touch / 2 AP)
//   0x0032  WiFi STATUS       link state (0 none / 1 router / 2 server / 3 touch / 4 AP)
//   0x0033  IPV4-H            high 16 bits of IPv4 address
//   0x0034  IPV4-L            low 16 bits of IPv4 address

// ─── Memory groups M0–M9 ─────────────────────────────────────────────────────
//
// 10 preset groups, 14 registers each, base 0x0050, stride 0x0010.
// M0 is the live operating set.

pub const GROUP_BASE: u16 = 0x0050;
pub const GROUP_STRIDE: u16 = 0x0010;
/// Number of registers per memory group.
pub const GROUP_LEN: u16 = 14;
/// Number of memory groups (M0 through M9).
pub const GROUP_COUNT: u8 = 10;

/// Base register address of memory group `n` (0..=9).
pub const fn group_addr(n: u8) -> u16 {
    GROUP_BASE + (n as u16) * GROUP_STRIDE
}

// In-group register offsets (add to `group_addr(n)` to get an absolute address).
pub const GROUP_OFF_V_SET: u16 = 0;
pub const GROUP_OFF_I_SET: u16 = 1;
pub const GROUP_OFF_S_LVP: u16 = 2;
pub const GROUP_OFF_S_OVP: u16 = 3;
pub const GROUP_OFF_S_OCP: u16 = 4;
pub const GROUP_OFF_S_OPP: u16 = 5;
pub const GROUP_OFF_S_OHP_H: u16 = 6;
pub const GROUP_OFF_S_OHP_M: u16 = 7;
pub const GROUP_OFF_S_OAH_L: u16 = 8;
pub const GROUP_OFF_S_OAH_H: u16 = 9;
pub const GROUP_OFF_S_OWH_L: u16 = 10;
pub const GROUP_OFF_S_OWH_H: u16 = 11;
pub const GROUP_OFF_S_OTP: u16 = 12;
pub const GROUP_OFF_S_INI: u16 = 13;

// Convenience absolute addresses for the active group (M0).
pub const REG_S_LVP: u16 = GROUP_BASE + GROUP_OFF_S_LVP;
pub const REG_S_OVP: u16 = GROUP_BASE + GROUP_OFF_S_OVP;
pub const REG_S_OCP: u16 = GROUP_BASE + GROUP_OFF_S_OCP;
pub const REG_S_OPP: u16 = GROUP_BASE + GROUP_OFF_S_OPP;
pub const REG_S_OTP: u16 = GROUP_BASE + GROUP_OFF_S_OTP;
pub const REG_S_INI: u16 = GROUP_BASE + GROUP_OFF_S_INI;

// ─── Bulk-access layout invariants ───────────────────────────────────────────
//
// Every bulk read/write in `device::*` sends a single Modbus transaction
// over a contiguous register span. These asserts pin the adjacency the
// callers rely on — a typo'd address constant turns into a build error
// instead of a silent off-by-one at runtime.

// `read_setpoints` (V-SET, I-SET).
const _: () = assert!(REG_I_SET == REG_V_SET + 1);

// `read_status` (V-SET..OUTPUT_EN, 19 regs indexed by absolute address).
const _: () = assert!(REG_V_SET == 0);
const _: () = assert!(REG_V_OUT == REG_V_SET + 2);
const _: () = assert!(REG_I_OUT == REG_V_SET + 3);
const _: () = assert!(REG_P_OUT == REG_V_SET + 4);
const _: () = assert!(REG_V_IN == REG_V_SET + 5);
const _: () = assert!(REG_PROTECT == REG_V_SET + 0x10);
const _: () = assert!(REG_CVCC == REG_V_SET + 0x11);
const _: () = assert!(REG_OUTPUT_EN == REG_V_SET + 0x12);

// `read_totals` (AH_LOW..OUT_S, 7 regs).
const _: () = assert!(REG_AH_HIGH == REG_AH_LOW + 1);
const _: () = assert!(REG_WH_LOW == REG_AH_LOW + 2);
const _: () = assert!(REG_WH_HIGH == REG_AH_LOW + 3);
const _: () = assert!(REG_OUT_H == REG_AH_LOW + 4);
const _: () = assert!(REG_OUT_M == REG_AH_LOW + 5);
const _: () = assert!(REG_OUT_S == REG_AH_LOW + 6);

// `set_protection` / `read_protection` (S-LVP, S-OVP, S-OCP).
const _: () = assert!(REG_S_OVP == REG_S_LVP + 1);
const _: () = assert!(REG_S_OCP == REG_S_LVP + 2);

// `read_temperatures` (T_IN, T_EX).
const _: () = assert!(REG_T_EX == REG_T_IN + 1);

// `read_group` / `write_group` walk all 14 in-group offsets in order.
const _: () = assert!(GROUP_OFF_V_SET == 0);
const _: () = assert!(GROUP_OFF_I_SET == 1);
const _: () = assert!(GROUP_OFF_S_LVP == 2);
const _: () = assert!(GROUP_OFF_S_OVP == 3);
const _: () = assert!(GROUP_OFF_S_OCP == 4);
const _: () = assert!(GROUP_OFF_S_OPP == 5);
const _: () = assert!(GROUP_OFF_S_OHP_H == 6);
const _: () = assert!(GROUP_OFF_S_OHP_M == 7);
const _: () = assert!(GROUP_OFF_S_OAH_L == 8);
const _: () = assert!(GROUP_OFF_S_OAH_H == 9);
const _: () = assert!(GROUP_OFF_S_OWH_L == 10);
const _: () = assert!(GROUP_OFF_S_OWH_H == 11);
const _: () = assert!(GROUP_OFF_S_OTP == 12);
const _: () = assert!(GROUP_OFF_S_INI == 13);
const _: () = assert!(GROUP_LEN == 14);
