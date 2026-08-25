//! Errors, grouped by who is at fault.
//!
//! [`RtuError`] covers everything the protocol stack can report — framing,
//! transport, and the bundled UART all return it. Its variants fall into two
//! classes, separated by [`RtuError::is_request_error`]: a malformed request,
//! which means a bug in the calling code and will fail identically on retry,
//! and a bus or device fault, which a supervisor may reasonably retry.
//!
//! [`InputError`] is the high-level device API rejecting a *value* before it
//! reaches the wire, and names the [`InputField`] at fault. [`XyError`] is the
//! union the [`crate::Xy`] methods return.

use core::fmt;

/// UART operation that failed.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IoOperation {
    Read,
    Write,
    Flush,
}

impl fmt::Display for IoOperation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Read => "read",
            Self::Write => "write",
            Self::Flush => "flush",
        })
    }
}

/// Portable classification of an underlying transport I/O failure.
///
/// The bundled UART maps every known `embedded_io::ErrorKind` into this type;
/// future upstream categories fall back to [`IoErrorKind::Other`]. Keeping the
/// classification here lets custom transports use the error API without
/// enabling the optional `embedded-io` dependency.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IoErrorKind {
    Other,
    NotFound,
    PermissionDenied,
    ConnectionRefused,
    ConnectionReset,
    ConnectionAborted,
    NotConnected,
    AddrInUse,
    AddrNotAvailable,
    BrokenPipe,
    AlreadyExists,
    InvalidInput,
    InvalidData,
    TimedOut,
    Interrupted,
    Unsupported,
    OutOfMemory,
    WriteZero,
}

impl fmt::Display for IoErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

#[cfg(feature = "embedded-io")]
impl From<embedded_io::ErrorKind> for IoErrorKind {
    fn from(kind: embedded_io::ErrorKind) -> Self {
        match kind {
            embedded_io::ErrorKind::Other => Self::Other,
            embedded_io::ErrorKind::NotFound => Self::NotFound,
            embedded_io::ErrorKind::PermissionDenied => Self::PermissionDenied,
            embedded_io::ErrorKind::ConnectionRefused => Self::ConnectionRefused,
            embedded_io::ErrorKind::ConnectionReset => Self::ConnectionReset,
            embedded_io::ErrorKind::ConnectionAborted => Self::ConnectionAborted,
            embedded_io::ErrorKind::NotConnected => Self::NotConnected,
            embedded_io::ErrorKind::AddrInUse => Self::AddrInUse,
            embedded_io::ErrorKind::AddrNotAvailable => Self::AddrNotAvailable,
            embedded_io::ErrorKind::BrokenPipe => Self::BrokenPipe,
            embedded_io::ErrorKind::AlreadyExists => Self::AlreadyExists,
            embedded_io::ErrorKind::InvalidInput => Self::InvalidInput,
            embedded_io::ErrorKind::InvalidData => Self::InvalidData,
            embedded_io::ErrorKind::TimedOut => Self::TimedOut,
            embedded_io::ErrorKind::Interrupted => Self::Interrupted,
            embedded_io::ErrorKind::Unsupported => Self::Unsupported,
            embedded_io::ErrorKind::OutOfMemory => Self::OutOfMemory,
            embedded_io::ErrorKind::WriteZero => Self::WriteZero,
            _ => Self::Other,
        }
    }
}

/// Anything the Modbus-RTU stack can report — framing, transport, and the
/// bundled UART all return this.
///
/// The first four variants mean the *request* was malformed and never reached
/// the bus; the rest mean the bus or the device misbehaved.
/// [`Self::is_request_error`] separates them, which is the distinction a retry
/// loop needs.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtuError {
    /// Zero registers requested, or more than the function code allows.
    InvalidQuantity(usize),
    /// Slave address outside the Modbus range `0..=247`.
    InvalidSlaveAddress(u8),
    /// A read used the broadcast address `0`, which no slave answers.
    BroadcastRead,
    /// The supplied buffer was smaller than the assembled frame.
    BufferTooSmall { needed: usize, actual: usize },
    /// Response was shorter than the expected reply.
    ShortResponse(usize),
    /// Slave address byte in the response didn't match the request.
    BadSlave(u8),
    /// Function code, byte count, address, quantity, or frame length didn't
    /// match what was expected.
    BadHeader,
    /// CRC-16 mismatch.
    BadCrc,
    /// Slave returned a Modbus exception. The byte is the exception code
    /// (`0x01`–`0x0B` per the spec).
    Exception(u8),
    /// No (or insufficient) bytes received within the response window.
    Timeout,
    /// RX activity prevented acquisition of a complete pre-transmit quiet gap.
    BusBusy,
    /// Underlying transport returned an I/O error.
    Io {
        operation: IoOperation,
        kind: IoErrorKind,
    },
}

impl RtuError {
    /// Whether the *request* was malformed — a bug in the calling code, which
    /// will fail identically on retry. `false` means a bus or device fault,
    /// which a supervisor may reasonably retry.
    pub const fn is_request_error(self) -> bool {
        matches!(
            self,
            Self::InvalidQuantity(_)
                | Self::InvalidSlaveAddress(_)
                | Self::BroadcastRead
                | Self::BufferTooSmall { .. }
        )
    }
}

impl fmt::Display for RtuError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidQuantity(n) => write!(f, "invalid register quantity {n}"),
            Self::InvalidSlaveAddress(address) => {
                write!(f, "invalid Modbus slave address {address}")
            }
            Self::BroadcastRead => f.write_str("read request cannot use broadcast address 0"),
            Self::BufferTooSmall { needed, actual } => {
                write!(f, "buffer too small (need {needed}, have {actual})")
            }
            Self::ShortResponse(n) => write!(f, "short response ({n} bytes)"),
            Self::BadSlave(a) => write!(f, "wrong slave id 0x{a:02X}"),
            Self::BadHeader => f.write_str("malformed header"),
            Self::BadCrc => f.write_str("CRC mismatch"),
            Self::Exception(c) => write!(f, "modbus exception 0x{c:02X}"),
            Self::Timeout => f.write_str("UART response timed out"),
            Self::BusBusy => f.write_str("UART bus did not become quiet"),
            Self::Io { operation, kind } => write!(f, "UART {operation} error ({kind})"),
        }
    }
}

impl core::error::Error for RtuError {}

/// High-level API field rejected during input validation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

/// A value the high-level device API rejected before it reached the wire.
///
/// Modbus-level request faults — a bad slave address or register quantity —
/// are [`RtuError`] instead, so each condition has exactly one variant.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InputError {
    NonFinite { field: InputField },
    OutOfRange { field: InputField },
    VoltageSetpointAboveProtection,
}

impl fmt::Display for InputError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite { field } => write!(f, "{field} must be finite"),
            Self::OutOfRange { field } => write!(f, "{field} is out of range"),
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
    /// A value the caller supplied is unusable.
    Input(InputError),
    /// The device returned a word this driver cannot interpret.
    InvalidRegisterValue { register: u16, value: u16 },
    /// The protocol stack failed.
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
