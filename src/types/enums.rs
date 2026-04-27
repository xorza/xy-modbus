//! Wire-encoded status enums (regulation mode, temperature unit,
//! protection cause, baud-rate code).

use core::fmt;

// ─── RegMode ─────────────────────────────────────────────────────────────────

/// Regulation mode reported by `CVCC` (register 0x0011).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RegMode {
    ConstantVoltage,
    ConstantCurrent,
}

impl RegMode {
    pub const fn from_reg(v: u16) -> Self {
        match v {
            0 => Self::ConstantVoltage,
            _ => Self::ConstantCurrent,
        }
    }
}

// ─── TempUnit ────────────────────────────────────────────────────────────────

/// Temperature unit selected by `F-C` (register 0x0013).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TempUnit {
    Celsius,
    Fahrenheit,
}

impl TempUnit {
    pub const fn from_reg(v: u16) -> Self {
        match v {
            0 => Self::Celsius,
            _ => Self::Fahrenheit,
        }
    }
    pub const fn to_reg(self) -> u16 {
        match self {
            Self::Celsius => 0,
            Self::Fahrenheit => 1,
        }
    }
}

// ─── ProtectionStatus ────────────────────────────────────────────────────────

/// Latched protection cause read from `PROTECT` (register 0x0010).
///
/// `Normal` (0) is the only non-tripped state. The register stays
/// latched until written back to 0.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ProtectionStatus {
    /// Operating normally.
    Normal,
    /// Output overvoltage. Also fires transiently when V-SET is raised
    /// above the current S-OVP threshold — program protection before
    /// raising V-SET.
    Ovp,
    /// Output overcurrent.
    Ocp,
    /// Output overpower.
    Opp,
    /// Input under-voltage (LVP setpoint).
    Lvp,
    /// Cumulative charge limit reached.
    Oah,
    /// Output-on time limit reached.
    Ohp,
    /// Over-temperature.
    Otp,
    /// Cumulative energy (Ah) limit reached.
    Oep,
    /// Cumulative energy (Wh) limit reached.
    Owh,
    /// Input over-current / inrush.
    Icp,
    /// Register read back a value outside the documented 0–10 range.
    Unknown(u16),
}

impl ProtectionStatus {
    pub const fn from_reg(raw: u16) -> Self {
        match raw {
            0 => Self::Normal,
            1 => Self::Ovp,
            2 => Self::Ocp,
            3 => Self::Opp,
            4 => Self::Lvp,
            5 => Self::Oah,
            6 => Self::Ohp,
            7 => Self::Otp,
            8 => Self::Oep,
            9 => Self::Owh,
            10 => Self::Icp,
            other => Self::Unknown(other),
        }
    }
}

impl fmt::Display for ProtectionStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Normal => "normal",
            Self::Ovp => "ovp",
            Self::Ocp => "ocp",
            Self::Opp => "opp",
            Self::Lvp => "lvp",
            Self::Oah => "oah",
            Self::Ohp => "ohp",
            Self::Otp => "otp",
            Self::Oep => "oep",
            Self::Owh => "owh",
            Self::Icp => "icp",
            Self::Unknown(v) => return write!(f, "unknown({v})"),
        })
    }
}

// ─── BaudRate ────────────────────────────────────────────────────────────────

/// Baud-rate codes for `BAUDRATE_L` (register 0x0019).
///
/// Only `B115200` (code 6) is documented in the seller manual; codes
/// 0–5 and 7–8 are community-derived. Verify on your unit before
/// committing a write. Baud changes take effect after device reset.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BaudRate {
    B9600,
    B14400,
    B19200,
    B38400,
    B56000,
    B57600,
    B115200,
    B2400,
    B4800,
    /// Register read back a code outside the documented 0–8 range.
    Unknown(u16),
}

impl BaudRate {
    /// Encoded register value. `Unknown(c)` round-trips its raw code.
    pub const fn code(self) -> u16 {
        match self {
            Self::B9600 => 0,
            Self::B14400 => 1,
            Self::B19200 => 2,
            Self::B38400 => 3,
            Self::B56000 => 4,
            Self::B57600 => 5,
            Self::B115200 => 6,
            Self::B2400 => 7,
            Self::B4800 => 8,
            Self::Unknown(c) => c,
        }
    }
    pub const fn from_code(code: u16) -> Self {
        match code {
            0 => Self::B9600,
            1 => Self::B14400,
            2 => Self::B19200,
            3 => Self::B38400,
            4 => Self::B56000,
            5 => Self::B57600,
            6 => Self::B115200,
            7 => Self::B2400,
            8 => Self::B4800,
            c => Self::Unknown(c),
        }
    }
    /// Bits-per-second, or `None` for `Unknown`.
    pub const fn baud(self) -> Option<u32> {
        Some(match self {
            Self::B2400 => 2400,
            Self::B4800 => 4800,
            Self::B9600 => 9600,
            Self::B14400 => 14400,
            Self::B19200 => 19200,
            Self::B38400 => 38400,
            Self::B56000 => 56000,
            Self::B57600 => 57600,
            Self::B115200 => 115200,
            Self::Unknown(_) => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::format;

    /// Pin every documented protection code (0..=10) plus an out-of-range
    /// case. A reordering of the match arms in `from_reg` would surface
    /// here.
    #[test]
    fn protection_status_from_reg_full_mapping() {
        let cases = [
            (0, ProtectionStatus::Normal),
            (1, ProtectionStatus::Ovp),
            (2, ProtectionStatus::Ocp),
            (3, ProtectionStatus::Opp),
            (4, ProtectionStatus::Lvp),
            (5, ProtectionStatus::Oah),
            (6, ProtectionStatus::Ohp),
            (7, ProtectionStatus::Otp),
            (8, ProtectionStatus::Oep),
            (9, ProtectionStatus::Owh),
            (10, ProtectionStatus::Icp),
            (11, ProtectionStatus::Unknown(11)),
            (0xFFFF, ProtectionStatus::Unknown(0xFFFF)),
        ];
        for (raw, expected) in cases {
            assert_eq!(ProtectionStatus::from_reg(raw), expected);
        }
    }

    /// Display strings are part of the public API (used in logs); pin them.
    #[test]
    fn protection_status_display_strings() {
        assert_eq!(format!("{}", ProtectionStatus::Normal), "normal");
        assert_eq!(format!("{}", ProtectionStatus::Ovp), "ovp");
        assert_eq!(format!("{}", ProtectionStatus::Icp), "icp");
        assert_eq!(format!("{}", ProtectionStatus::Unknown(42)), "unknown(42)");
    }

    /// `code()` and `from_code()` must invert each other across the full
    /// 0..=8 range, and `Unknown(c)` must round-trip arbitrary codes.
    /// `baud()` returns the documented bits-per-second.
    #[test]
    fn baud_rate_full_table() {
        let cases = [
            (0, BaudRate::B9600, 9600),
            (1, BaudRate::B14400, 14400),
            (2, BaudRate::B19200, 19200),
            (3, BaudRate::B38400, 38400),
            (4, BaudRate::B56000, 56000),
            (5, BaudRate::B57600, 57600),
            (6, BaudRate::B115200, 115200),
            (7, BaudRate::B2400, 2400),
            (8, BaudRate::B4800, 4800),
        ];
        for (code, variant, bps) in cases {
            assert_eq!(BaudRate::from_code(code), variant);
            assert_eq!(variant.code(), code);
            assert_eq!(variant.baud(), Some(bps));
        }
        assert_eq!(BaudRate::from_code(99), BaudRate::Unknown(99));
        assert_eq!(BaudRate::Unknown(99).code(), 99);
        assert_eq!(BaudRate::Unknown(99).baud(), None);
    }

    #[test]
    fn temp_unit_round_trip() {
        assert_eq!(TempUnit::from_reg(0), TempUnit::Celsius);
        assert_eq!(TempUnit::from_reg(1), TempUnit::Fahrenheit);
        // Any nonzero value decodes to Fahrenheit.
        assert_eq!(TempUnit::from_reg(99), TempUnit::Fahrenheit);
        assert_eq!(TempUnit::Celsius.to_reg(), 0);
        assert_eq!(TempUnit::Fahrenheit.to_reg(), 1);
    }
}
