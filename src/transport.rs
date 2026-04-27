//! Modbus-RTU transport trait and error types.
//!
//! Implement [`ModbusTransport`] over your platform's UART. The
//! [`crate::framing`] module gives you the on-wire codec; a typical
//! implementation is <100 lines of UART-specific timing on top.

use core::fmt;

// ─── ModbusError ─────────────────────────────────────────────────────────────

/// Protocol-layer error: a frame was received but failed validation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModbusError {
    /// Response was shorter than the smallest valid frame for the
    /// expected reply.
    ShortResponse(usize),
    /// Slave address byte didn't match the request.
    BadSlave(u8),
    /// Function-code, byte-count, address, or quantity field didn't
    /// match what was expected.
    BadHeader,
    /// CRC-16 mismatch.
    BadCrc,
    /// Slave returned a Modbus exception. The byte is the exception
    /// code (`0x01`–`0x0B` per the spec).
    Exception(u8),
}

impl fmt::Display for ModbusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ShortResponse(n) => write!(f, "short response ({n} bytes)"),
            Self::BadSlave(a) => write!(f, "wrong slave id 0x{a:02X}"),
            Self::BadHeader => write!(f, "malformed header"),
            Self::BadCrc => write!(f, "CRC mismatch"),
            Self::Exception(c) => write!(f, "modbus exception 0x{c:02X}"),
        }
    }
}

impl core::error::Error for ModbusError {}

// ─── RtuError ────────────────────────────────────────────────────────────────

/// Unified error returned by the device API: either the transport
/// (UART layer) failed, or the response was a malformed / exception
/// Modbus frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtuError {
    /// No (or insufficient) bytes received within the response window.
    Timeout,
    /// Underlying UART returned an I/O error on read or write.
    Io,
    /// Decoded response was invalid or the slave reported a Modbus
    /// exception.
    Modbus(ModbusError),
}

impl fmt::Display for RtuError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => f.write_str("UART response timed out"),
            Self::Io => f.write_str("UART I/O error"),
            Self::Modbus(e) => fmt::Display::fmt(e, f),
        }
    }
}

impl From<ModbusError> for RtuError {
    fn from(e: ModbusError) -> Self {
        Self::Modbus(e)
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

// ─── BlockingRead ────────────────────────────────────────────────────────────

/// Read side of the bundled [`crate::uart::UartTransport`].
///
/// Wraps a UART driver that already knows how to block efficiently for
/// incoming bytes — every kernel-backed HAL exposes one
/// (`esp_idf_hal::uart::UartDriver::read(buf, ticks)`, embassy with
/// timeout futures, `serialport-rs::SerialPort::read` after
/// `set_timeout`, …). Implementing this trait is typically 3 lines that
/// translate `timeout_ms` into the driver's native timeout type.
///
/// Avoiding `embedded_io::ReadReady` here is deliberate: the trait is
/// optional in the embedded-io ecosystem and many HALs (esp-idf-hal
/// included) don't impl it, and the busy-poll loop a `ReadReady`-based
/// implementation needs fights kernel-backed drivers that already block
/// cheaply.
pub trait BlockingRead {
    type Error;

    /// Block for up to `timeout_ms` waiting for at least one byte to
    /// arrive, then return up to `buf.len()` bytes. `Ok(0)` means the
    /// timeout elapsed without a byte appearing. `timeout_ms == 0` is a
    /// non-blocking poll: return whatever is already buffered without
    /// waiting (used for the pre-TX flush).
    fn read(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, Self::Error>;
}

// ─── Transport trait ─────────────────────────────────────────────────────────

/// Modbus-RTU transport: send a request, validate the response, hand
/// back the payload (for reads) or just `Ok(())` (for writes).
///
/// Implementers handle UART framing timing — the inter-frame gap, the
/// per-device response timeout, and the post-write quiet gap. The
/// XY-series wants ~50 ms between frames and ~500 ms response window.
///
/// All three function codes are required; the device API uses each
/// (`0x03` for reads, `0x06` for single setpoint writes, `0x10` for
/// bulk memory-group writes).
pub trait ModbusTransport {
    fn read_holding(&mut self, slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError>;

    fn write_single_holding(&mut self, slave: u8, addr: u16, value: u16) -> Result<(), RtuError>;

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
    fn modbus_error_display_strings() {
        assert_eq!(
            format!("{}", ModbusError::ShortResponse(3)),
            "short response (3 bytes)"
        );
        assert_eq!(
            format!("{}", ModbusError::BadSlave(0x02)),
            "wrong slave id 0x02"
        );
        assert_eq!(format!("{}", ModbusError::BadHeader), "malformed header");
        assert_eq!(format!("{}", ModbusError::BadCrc), "CRC mismatch");
        assert_eq!(
            format!("{}", ModbusError::Exception(0x03)),
            "modbus exception 0x03"
        );
    }

    #[test]
    fn rtu_error_display_strings() {
        assert_eq!(format!("{}", RtuError::Timeout), "UART response timed out");
        assert_eq!(format!("{}", RtuError::Io), "UART I/O error");
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
        let e = RtuError::Modbus(ModbusError::BadCrc);
        assert!(e.source().is_some());
        assert!(RtuError::Timeout.source().is_none());
        assert!(RtuError::Io.source().is_none());
    }
}
