use core::fmt;

use crate::transport::RtuError;

/// High-level API field rejected during input validation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum InputField {
    VoltageSetpoint,
    CurrentSetpoint,
    LowVoltageProtection,
    OverVoltageProtection,
    OverCurrentProtection,
    OverPowerProtection,
    OverTemperatureProtection,
    OutputTimeHours,
    OutputTimeMinutes,
    ChargeLimit,
    EnergyLimit,
    Backlight,
    SleepTimeout,
}

impl fmt::Display for InputField {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::VoltageSetpoint => "voltage setpoint",
            Self::CurrentSetpoint => "current setpoint",
            Self::LowVoltageProtection => "low-voltage protection",
            Self::OverVoltageProtection => "over-voltage protection",
            Self::OverCurrentProtection => "over-current protection",
            Self::OverPowerProtection => "over-power protection",
            Self::OverTemperatureProtection => "over-temperature protection",
            Self::OutputTimeHours => "output-time hours",
            Self::OutputTimeMinutes => "output-time minutes",
            Self::ChargeLimit => "charge limit",
            Self::EnergyLimit => "energy limit",
            Self::Backlight => "backlight",
            Self::SleepTimeout => "sleep timeout",
        })
    }
}

/// Invalid value supplied to the high-level device API.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum InputError {
    NonFinite { field: InputField },
    OutOfRange { field: InputField },
    InvalidSlaveAddress { address: u8 },
    InvalidGroup { group: u8 },
    VoltageSetpointAboveProtection,
}

impl fmt::Display for InputError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite { field } => write!(f, "{field} must be finite"),
            Self::OutOfRange { field } => write!(f, "{field} is out of range"),
            Self::InvalidSlaveAddress { address } => {
                write!(f, "invalid Modbus slave address {address}")
            }
            Self::InvalidGroup { group } => write!(f, "invalid memory group {group}"),
            Self::VoltageSetpointAboveProtection => {
                f.write_str("voltage setpoint exceeds over-voltage protection")
            }
        }
    }
}

impl core::error::Error for InputError {}

/// Error returned by the high-level XY device API.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum XyError {
    Input(InputError),
    InvalidRegisterValue { register: u16, value: u16 },
    Rtu(RtuError),
}

impl fmt::Display for XyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Input(error) => fmt::Display::fmt(error, f),
            Self::InvalidRegisterValue { register, value } => {
                write!(f, "invalid value {value} in register 0x{register:04X}")
            }
            Self::Rtu(error) => fmt::Display::fmt(error, f),
        }
    }
}

impl From<InputError> for XyError {
    fn from(error: InputError) -> Self {
        Self::Input(error)
    }
}

impl From<RtuError> for XyError {
    fn from(error: RtuError) -> Self {
        Self::Rtu(error)
    }
}

impl core::error::Error for XyError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Input(error) => Some(error),
            Self::Rtu(error) => Some(error),
            Self::InvalidRegisterValue { .. } => None,
        }
    }
}

#[cfg(test)]
mod tests;
