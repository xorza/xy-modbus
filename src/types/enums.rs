//! Wire-encoded status enums (regulation mode, temperature unit,
//! protection cause, baud-rate code).

use core::fmt;

/// Regulation mode reported by `CVCC` (register 0x0011).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RegMode {
    ConstantVoltage,
    ConstantCurrent,
}

impl RegMode {
    pub(crate) const fn from_reg(v: u16) -> Result<Self, u16> {
        Ok(match v {
            0 => Self::ConstantVoltage,
            1 => Self::ConstantCurrent,
            invalid => return Err(invalid),
        })
    }
}

/// Temperature unit selected by `F-C` (register 0x0013).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TempUnit {
    Celsius,
    Fahrenheit,
}

impl TempUnit {
    pub(crate) const fn from_reg(v: u16) -> Result<Self, u16> {
        Ok(match v {
            0 => Self::Celsius,
            1 => Self::Fahrenheit,
            invalid => return Err(invalid),
        })
    }
    pub(crate) const fn code(self) -> u16 {
        match self {
            Self::Celsius => 0,
            Self::Fahrenheit => 1,
        }
    }
}

/// Latched protection cause read from `PROTECT` (register 0x0010).
///
/// `Normal` (0) is the only non-tripped state. The register stays
/// latched until written back to 0.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// Internal power-stage/no-output protection. Exact triggers are
    /// model- and firmware-dependent and remain unverified on XY7025.
    Oep,
    /// Cumulative energy (Wh) limit reached.
    Owh,
    /// Input over-current / inrush.
    Icp,
}

impl ProtectionStatus {
    pub(crate) const fn from_reg(raw: u16) -> Result<Self, u16> {
        Ok(match raw {
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
            invalid => return Err(invalid),
        })
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
        })
    }
}

/// Baud-rate codes for `BAUDRATE_L` (register 0x0019).
///
/// Only `B115200` (code 6) is documented in the seller manual; codes
/// 0–5 and 7–8 are community-derived. Verify on your unit before
/// committing a write. Baud changes take effect after device reset.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
}

impl BaudRate {
    pub(crate) const fn code(self) -> u16 {
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
        }
    }
    pub(crate) const fn from_code(code: u16) -> Result<Self, u16> {
        Ok(match code {
            0 => Self::B9600,
            1 => Self::B14400,
            2 => Self::B19200,
            3 => Self::B38400,
            4 => Self::B56000,
            5 => Self::B57600,
            6 => Self::B115200,
            7 => Self::B2400,
            8 => Self::B4800,
            invalid => return Err(invalid),
        })
    }
    pub const fn baud(self) -> u32 {
        match self {
            Self::B2400 => 2400,
            Self::B4800 => 4800,
            Self::B9600 => 9600,
            Self::B14400 => 14400,
            Self::B19200 => 19200,
            Self::B38400 => 38400,
            Self::B56000 => 56000,
            Self::B57600 => 57600,
            Self::B115200 => 115200,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::format;

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
        ];
        for (raw, expected) in cases {
            assert_eq!(ProtectionStatus::from_reg(raw), Ok(expected));
        }
        assert_eq!(ProtectionStatus::from_reg(11), Err(11));
        assert_eq!(ProtectionStatus::from_reg(u16::MAX), Err(u16::MAX));
    }

    #[test]
    fn protection_status_display_strings() {
        assert_eq!(format!("{}", ProtectionStatus::Normal), "normal");
        assert_eq!(format!("{}", ProtectionStatus::Ovp), "ovp");
        assert_eq!(format!("{}", ProtectionStatus::Icp), "icp");
    }

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
            assert_eq!(BaudRate::from_code(code), Ok(variant));
            assert_eq!(variant.code(), code);
            assert_eq!(variant.baud(), bps);
        }
        assert_eq!(BaudRate::from_code(99), Err(99));
    }

    #[test]
    fn temp_unit_round_trip() {
        assert_eq!(TempUnit::from_reg(0), Ok(TempUnit::Celsius));
        assert_eq!(TempUnit::from_reg(1), Ok(TempUnit::Fahrenheit));
        assert_eq!(TempUnit::from_reg(99), Err(99));
        assert_eq!(TempUnit::Celsius.code(), 0);
        assert_eq!(TempUnit::Fahrenheit.code(), 1);
        assert_eq!(RegMode::from_reg(0), Ok(RegMode::ConstantVoltage));
        assert_eq!(RegMode::from_reg(1), Ok(RegMode::ConstantCurrent));
        assert_eq!(RegMode::from_reg(99), Err(99));
    }
}
