use core::fmt;

use crate::transport::RtuError;

/// Invalid value supplied to the high-level device API.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InputError {
    NonFinite { field: &'static str },
    OutOfRange { field: &'static str },
    InvalidSlaveAddress { address: u8 },
    InvalidGroup { group: u8 },
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
