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

/// Error returned by a Modbus-RTU transport operation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtuError {
    /// Caller requested zero registers or exceeded the function-code limit.
    InvalidQuantity(usize),
    /// RX activity prevented acquisition of a complete pre-transmit quiet gap.
    BusBusy,
    /// No (or insufficient) bytes received within the response window.
    Timeout,
    /// Underlying transport returned an I/O error.
    Io {
        operation: IoOperation,
        kind: IoErrorKind,
    },
    /// Decoded response was invalid or the slave reported a Modbus
    /// exception.
    Modbus(ModbusError),
}

impl fmt::Display for RtuError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidQuantity(n) => write!(f, "invalid register quantity {n}"),
            Self::BusBusy => f.write_str("UART bus did not become quiet"),
            Self::Timeout => f.write_str("UART response timed out"),
            Self::Io { operation, kind } => write!(f, "UART {operation} error ({kind})"),
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
/// Write operations may use slave address `0` for a Modbus broadcast. Because
/// broadcasts have no response, `Ok(())` then confirms transmission only, not
/// that any device accepted the write. Reads must use a unicast address.
///
/// Implementers handle UART framing timing — the inter-frame gap, the
/// per-device read timeout, and the post-write quiet gap. The bundled
/// UART transport uses ~50 ms between frames and a 500 ms inactivity
/// timeout for each partial read. Its default pre-transmit acquisition returns
/// [`RtuError::BusBusy`] after ten consecutive noisy intervals rather than
/// waiting indefinitely.
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
    /// Read `dst.len()` holding registers from a unicast slave.
    fn read_holding(&mut self, slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError>;

    /// Write one holding register; slave `0` broadcasts without acknowledgement.
    fn write_single_holding(&mut self, slave: u8, addr: u16, value: u16) -> Result<(), RtuError>;

    /// Write every holding register in `values`; slave `0` broadcasts without
    /// acknowledgement.
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
        assert_eq!(
            format!("{}", RtuError::BusBusy),
            "UART bus did not become quiet"
        );
        assert_eq!(format!("{}", RtuError::Timeout), "UART response timed out");
        assert_eq!(
            format!(
                "{}",
                RtuError::Io {
                    operation: IoOperation::Read,
                    kind: IoErrorKind::ConnectionReset,
                }
            ),
            "UART read error (ConnectionReset)"
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
        assert!(RtuError::BusBusy.source().is_none());
        assert!(RtuError::Timeout.source().is_none());
        assert!(
            RtuError::Io {
                operation: IoOperation::Read,
                kind: IoErrorKind::ConnectionReset,
            }
            .source()
            .is_none()
        );
    }

    #[cfg(feature = "embedded-io")]
    #[test]
    fn embedded_io_error_kinds_map_without_losing_detail() {
        let cases = [
            (embedded_io::ErrorKind::Other, IoErrorKind::Other),
            (embedded_io::ErrorKind::NotFound, IoErrorKind::NotFound),
            (
                embedded_io::ErrorKind::PermissionDenied,
                IoErrorKind::PermissionDenied,
            ),
            (
                embedded_io::ErrorKind::ConnectionRefused,
                IoErrorKind::ConnectionRefused,
            ),
            (
                embedded_io::ErrorKind::ConnectionReset,
                IoErrorKind::ConnectionReset,
            ),
            (
                embedded_io::ErrorKind::ConnectionAborted,
                IoErrorKind::ConnectionAborted,
            ),
            (
                embedded_io::ErrorKind::NotConnected,
                IoErrorKind::NotConnected,
            ),
            (embedded_io::ErrorKind::AddrInUse, IoErrorKind::AddrInUse),
            (
                embedded_io::ErrorKind::AddrNotAvailable,
                IoErrorKind::AddrNotAvailable,
            ),
            (embedded_io::ErrorKind::BrokenPipe, IoErrorKind::BrokenPipe),
            (
                embedded_io::ErrorKind::AlreadyExists,
                IoErrorKind::AlreadyExists,
            ),
            (
                embedded_io::ErrorKind::InvalidInput,
                IoErrorKind::InvalidInput,
            ),
            (
                embedded_io::ErrorKind::InvalidData,
                IoErrorKind::InvalidData,
            ),
            (embedded_io::ErrorKind::TimedOut, IoErrorKind::TimedOut),
            (
                embedded_io::ErrorKind::Interrupted,
                IoErrorKind::Interrupted,
            ),
            (
                embedded_io::ErrorKind::Unsupported,
                IoErrorKind::Unsupported,
            ),
            (
                embedded_io::ErrorKind::OutOfMemory,
                IoErrorKind::OutOfMemory,
            ),
            (embedded_io::ErrorKind::WriteZero, IoErrorKind::WriteZero),
        ];
        for (source, expected) in cases {
            assert_eq!(IoErrorKind::from(source), expected);
        }
    }
}
