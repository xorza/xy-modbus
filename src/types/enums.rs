//! Wire-encoded status enums (regulation mode, temperature unit,
//! protection cause, baud-rate code).
//!
//! Each variant carries its wire code as an explicit discriminant, so the
//! encode direction is a cast and the decode direction is a scan over `ALL`.
//! That leaves one hand-written table per type — the parts that are genuinely
//! data, like a display string or a bit rate — instead of a forward table and
//! a reverse table that can drift apart.

use core::fmt;

/// Regulation mode reported by `CVCC` (register 0x0011).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RegMode {
    ConstantVoltage = 0,
    ConstantCurrent = 1,
}

impl RegMode {
    /// Decode-scan source. `CVCC` is read-only, which is why this type has no
    /// `code` — there is nothing to write back.
    const ALL: [Self; 2] = [Self::ConstantVoltage, Self::ConstantCurrent];

    pub(crate) const fn from_reg(v: u16) -> Result<Self, u16> {
        let mut i = 0;
        while i < Self::ALL.len() {
            if Self::ALL[i] as u16 == v {
                return Ok(Self::ALL[i]);
            }
            i += 1;
        }
        Err(v)
    }
}

/// Temperature unit selected by `F-C` (register 0x0013).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TempUnit {
    Celsius = 0,
    Fahrenheit = 1,
}

impl TempUnit {
    /// Decode-scan source.
    const ALL: [Self; 2] = [Self::Celsius, Self::Fahrenheit];

    pub(crate) const fn from_reg(v: u16) -> Result<Self, u16> {
        let mut i = 0;
        while i < Self::ALL.len() {
            if Self::ALL[i] as u16 == v {
                return Ok(Self::ALL[i]);
            }
            i += 1;
        }
        Err(v)
    }

    pub(crate) const fn code(self) -> u16 {
        self as u16
    }
}

/// Latched protection cause read from `PROTECT` (register 0x0010).
///
/// `Normal` (0) is the only non-tripped state. The register stays
/// latched until written back to 0.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ProtectionStatus {
    /// Operating normally.
    Normal = 0,
    /// Output overvoltage. Also fires transiently when V-SET is raised
    /// above the current S-OVP threshold — program protection before
    /// raising V-SET.
    Ovp = 1,
    /// Output overcurrent.
    Ocp = 2,
    /// Output overpower.
    Opp = 3,
    /// Input under-voltage (LVP setpoint).
    Lvp = 4,
    /// Cumulative charge limit reached.
    Oah = 5,
    /// Output-on time limit reached.
    Ohp = 6,
    /// Over-temperature.
    Otp = 7,
    /// Internal power-stage/no-output protection. Exact triggers are
    /// model- and firmware-dependent and remain unverified on XY7025.
    Oep = 8,
    /// Cumulative energy (Wh) limit reached.
    Owh = 9,
    /// Input over-current / inrush.
    Icp = 10,
}

impl ProtectionStatus {
    /// Decode-scan source, in wire-code order.
    const ALL: [Self; 11] = [
        Self::Normal,
        Self::Ovp,
        Self::Ocp,
        Self::Opp,
        Self::Lvp,
        Self::Oah,
        Self::Ohp,
        Self::Otp,
        Self::Oep,
        Self::Owh,
        Self::Icp,
    ];

    /// Wire code written back to `PROTECT`. Only `Normal` is useful to write —
    /// it clears the latch.
    pub(crate) const fn code(self) -> u16 {
        self as u16
    }

    pub(crate) const fn from_reg(raw: u16) -> Result<Self, u16> {
        let mut i = 0;
        while i < Self::ALL.len() {
            if Self::ALL[i] as u16 == raw {
                return Ok(Self::ALL[i]);
            }
            i += 1;
        }
        Err(raw)
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
///
/// The discriminants are the wire codes, which is why the variants are not in
/// ascending bit-rate order — 2400 and 4800 were appended by the vendor as
/// codes 7 and 8.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BaudRate {
    B9600 = 0,
    B14400 = 1,
    B19200 = 2,
    B38400 = 3,
    B56000 = 4,
    B57600 = 5,
    B115200 = 6,
    B2400 = 7,
    B4800 = 8,
}

impl BaudRate {
    /// Decode-scan source, in wire-code order.
    const ALL: [Self; 9] = [
        Self::B9600,
        Self::B14400,
        Self::B19200,
        Self::B38400,
        Self::B56000,
        Self::B57600,
        Self::B115200,
        Self::B2400,
        Self::B4800,
    ];

    pub(crate) const fn code(self) -> u16 {
        self as u16
    }

    pub(crate) const fn from_code(code: u16) -> Result<Self, u16> {
        let mut i = 0;
        while i < Self::ALL.len() {
            if Self::ALL[i] as u16 == code {
                return Ok(Self::ALL[i]);
            }
            i += 1;
        }
        Err(code)
    }

    /// Bits per second. The one part of this type that is not derivable from
    /// the wire code.
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

    /// Every code round-trips, and `ALL` is ordered by code — the decode scan
    /// and the `as` cast must agree on all eleven.
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
            assert_eq!(expected.code(), raw);
        }
        assert_eq!(cases.len(), ProtectionStatus::ALL.len());
        for (i, status) in ProtectionStatus::ALL.iter().enumerate() {
            assert_eq!(status.code() as usize, i);
        }
        assert_eq!(ProtectionStatus::from_reg(11), Err(11));
        assert_eq!(ProtectionStatus::from_reg(u16::MAX), Err(u16::MAX));
    }

    #[test]
    fn protection_status_display_strings() {
        assert_eq!(format!("{}", ProtectionStatus::Normal), "normal");
        assert_eq!(format!("{}", ProtectionStatus::Ovp), "ovp");
        assert_eq!(format!("{}", ProtectionStatus::Icp), "icp");
        // Every variant renders distinctly.
        let mut seen: std::vec::Vec<std::string::String> = ProtectionStatus::ALL
            .iter()
            .map(|s| format!("{s}"))
            .collect();
        seen.sort();
        let before = seen.len();
        seen.dedup();
        assert_eq!(seen.len(), before);
    }

    /// The wire codes are deliberately not in ascending bit-rate order, so the
    /// discriminant and the rate are pinned independently.
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
        assert_eq!(cases.len(), BaudRate::ALL.len());
        assert_eq!(BaudRate::from_code(9), Err(9));
        assert_eq!(BaudRate::from_code(99), Err(99));
    }

    #[test]
    fn temp_unit_and_reg_mode_round_trip() {
        assert_eq!(TempUnit::from_reg(0), Ok(TempUnit::Celsius));
        assert_eq!(TempUnit::from_reg(1), Ok(TempUnit::Fahrenheit));
        assert_eq!(TempUnit::from_reg(2), Err(2));
        assert_eq!(TempUnit::from_reg(99), Err(99));
        assert_eq!(TempUnit::Celsius.code(), 0);
        assert_eq!(TempUnit::Fahrenheit.code(), 1);
        assert_eq!(RegMode::from_reg(0), Ok(RegMode::ConstantVoltage));
        assert_eq!(RegMode::from_reg(1), Ok(RegMode::ConstantCurrent));
        assert_eq!(RegMode::from_reg(2), Err(2));
        assert_eq!(RegMode::from_reg(99), Err(99));
    }
}
