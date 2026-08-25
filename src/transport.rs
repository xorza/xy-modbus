//! The Modbus-RTU transport trait.
//!
//! Implement [`ModbusTransport`] over your platform's UART. The
//! [`crate::framing`] module gives you the on-wire codec — including the
//! slave-address checks this trait requires — and a typical implementation is
//! <100 lines of UART-specific timing on top. Failures are reported as
//! [`crate::RtuError`].

use crate::error::RtuError;

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
/// Implementations must return [`RtuError::InvalidSlaveAddress`] before I/O for
/// addresses outside `0..=247`, and [`RtuError::BroadcastRead`] for a read
/// addressed to `0`.
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
