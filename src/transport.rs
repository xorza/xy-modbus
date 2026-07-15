//! Modbus-RTU transport trait and error types.
//!
//! Implement [`ModbusTransport`] over your platform's UART. The
//! [`crate::framing`] module gives you the on-wire codec; a typical
//! implementation is <100 lines of UART-specific timing on top.

use core::fmt;

use crate::framing::ModbusError;

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

/// Error returned by a Modbus-RTU transport operation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtuError {
    /// Caller requested zero registers or exceeded the function-code limit.
    InvalidQuantity(usize),
    /// No (or insufficient) bytes received within the response window.
    Timeout,
    /// Underlying UART returned an I/O error.
    Io { operation: IoOperation },
    /// Decoded response was invalid or the slave reported a Modbus
    /// exception.
    Modbus(ModbusError),
}

impl fmt::Display for RtuError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidQuantity(n) => write!(f, "invalid register quantity {n}"),
            Self::Timeout => f.write_str("UART response timed out"),
            Self::Io { operation } => write!(f, "UART {operation} error"),
            Self::Modbus(e) => fmt::Display::fmt(e, f),
        }
    }
}

impl From<ModbusError> for RtuError {
    fn from(e: ModbusError) -> Self {
        match e {
            ModbusError::InvalidQuantity(quantity) => Self::InvalidQuantity(quantity),
            error => Self::Modbus(error),
        }
    }
}

impl core::error::Error for RtuError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Modbus(e) => Some(e),
            _ => None,
        }
    }
}

/// Modbus-RTU transport: send a request, validate the response, hand
/// back the payload (for reads) or just `Ok(())` (for writes).
///
/// Implementers handle UART framing timing — the inter-frame gap, the
/// per-device read timeout, and the post-write quiet gap. The bundled
/// UART transport uses ~50 ms between frames and a 500 ms inactivity
/// timeout for each partial read.
///
/// All three function codes are required; the device API uses each
/// (`0x03` for reads, `0x06` for single setpoint writes, `0x10` for
/// bulk memory-group writes).
///
/// Implementations must return [`RtuError::InvalidQuantity`] before I/O when a
/// read destination is empty or exceeds [`crate::framing::MAX_READ_REGS`], or
/// when a multi-write source is empty or exceeds
/// [`crate::framing::MAX_WRITE_REGS`].
pub trait ModbusTransport {
    /// Read `dst.len()` holding registers.
    fn read_holding(&mut self, slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError>;

    fn write_single_holding(&mut self, slave: u8, addr: u16, value: u16) -> Result<(), RtuError>;

    /// Write every holding register in `values`.
    fn write_multiple_holdings(
        &mut self,
        slave: u8,
        addr: u16,
        values: &[u16],
    ) -> Result<(), RtuError>;
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::format;

    #[test]
    fn rtu_error_display_strings() {
        assert_eq!(
            format!("{}", RtuError::InvalidQuantity(0)),
            "invalid register quantity 0"
        );
        assert_eq!(format!("{}", RtuError::Timeout), "UART response timed out");
        assert_eq!(
            format!(
                "{}",
                RtuError::Io {
                    operation: IoOperation::Read
                }
            ),
            "UART read error"
        );
        // Modbus variant delegates to inner Display.
        assert_eq!(
            format!("{}", RtuError::Modbus(ModbusError::BadCrc)),
            "CRC mismatch"
        );
    }

    /// Modbus-variant `RtuError` exposes the underlying error via `Error::source`.
    #[test]
    fn rtu_error_source_chain() {
        use core::error::Error;
        assert_eq!(
            RtuError::from(ModbusError::InvalidQuantity(126)),
            RtuError::InvalidQuantity(126)
        );
        let e = RtuError::Modbus(ModbusError::BadCrc);
        assert!(e.source().is_some());
        assert!(RtuError::InvalidQuantity(0).source().is_none());
        assert!(RtuError::Timeout.source().is_none());
        assert!(
            RtuError::Io {
                operation: IoOperation::Read
            }
            .source()
            .is_none()
        );
    }
}
