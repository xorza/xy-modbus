//! Default Modbus-RTU transport over a [`BlockingRead`] +
//! [`embedded_io::Write`] UART.
//!
//! Wrap any UART driver that exposes a blocking-with-timeout read (every
//! kernel-backed HAL does) plus an [`embedded_hal::delay::DelayNs`] timer
//! for the inter-frame silence, and you have a working
//! [`ModbusTransport`].
//!
//! ```ignore
//! use xy_modbus::{Xy, uart::UartTransport};
//!
//! let transport = UartTransport::new(uart, delay);
//! let mut xy = Xy::new(transport, Model::Xy7025);
//! ```
//!
//! Timing defaults match the XY-series spec (~500 ms response window,
//! ~50 ms post-write quiet gap). Override with [`UartTransport::with_timing`].

use embedded_hal::delay::DelayNs;
use embedded_io::Write;

use crate::framing::{
    EXCEPTION_BIT, MAX_ADU, MAX_READ_REGS, MAX_WRITE_REGS, build_read_request,
    build_write_multiple_request, build_write_single_request, parse_read_response,
    parse_write_multiple_response, parse_write_single_response,
};
use crate::transport::{BlockingRead, ModbusTransport, RtuError};

// `write_multiple_holdings` builds the request frame into `self.buf` and
// `expect`s success — sound only because the buffer fits the largest
// possible Write Multiple frame (slave + fc + addr + qty + bc + 2*regs + crc).
const _: () = assert!(MAX_ADU >= 9 + 2 * MAX_WRITE_REGS);

// ─── UartTransport ───────────────────────────────────────────────────────────

/// Generic Modbus-RTU transport over any blocking-with-timeout UART.
///
/// Holds a single [`MAX_ADU`]-sized scratch buffer reused across
/// transactions — keeps the per-call stack frame small (each
/// `read_holding` / `write_multiple_holdings` would otherwise allocate
/// 256 B locally).
#[derive(Debug)]
pub struct UartTransport<U, D> {
    uart: U,
    delay: D,
    response_timeout_ms: u32,
    inter_frame_ms: u32,
    buf: [u8; MAX_ADU],
}

impl<U, D> UartTransport<U, D>
where
    U: BlockingRead + Write,
    D: DelayNs,
{
    /// Build a transport with default XY-series timing
    /// (500 ms response window, 50 ms inter-frame quiet gap).
    pub fn new(uart: U, delay: D) -> Self {
        Self {
            uart,
            delay,
            response_timeout_ms: 500,
            inter_frame_ms: 50,
            buf: [0u8; MAX_ADU],
        }
    }

    /// Override the response timeout (max wait without any byte arriving)
    /// and the inter-frame quiet gap.
    pub fn with_timing(mut self, response_timeout_ms: u32, inter_frame_ms: u32) -> Self {
        self.response_timeout_ms = response_timeout_ms;
        self.inter_frame_ms = inter_frame_ms;
        self
    }

    /// Recover the inner UART and delay.
    pub fn release(self) -> (U, D) {
        (self.uart, self.delay)
    }

    /// Enforce ≥t3.5 bus silence before the next master frame, then
    /// flush any noise that arrived during the gap.
    fn pre_tx_silence(&mut self) {
        self.delay.delay_ms(self.inter_frame_ms);
        drain_rx(&mut self.uart);
    }
}

// ─── Free helpers (take `&mut U` so the buffer in `UartTransport` ──────────
//                  can be borrowed disjointly with the UART)

/// Discard whatever is already in the UART RX buffer. Cheap
/// non-blocking calls (`timeout_ms = 0`) drain noise that arrived during
/// the inter-frame silence so it doesn't masquerade as the start of the
/// slave's reply.
///
/// Capped at `DRAIN_MAX_BYTES` total — a well-behaved UART hits Ok(0) or
/// Err(…) within a few iterations, but a stuck driver returning >0
/// forever would otherwise loop indefinitely. We'd rather give up
/// draining and let the caller's exception/CRC checks fail than wedge.
fn drain_rx<U: BlockingRead>(uart: &mut U) {
    const DRAIN_MAX_BYTES: usize = 4 * MAX_ADU;
    let mut scratch = [0u8; 32];
    let mut drained = 0;
    while drained < DRAIN_MAX_BYTES {
        match uart.read(&mut scratch, 0) {
            Ok(0) | Err(_) => return,
            Ok(n) => drained += n,
        }
    }
}

fn write_all<U: Write>(uart: &mut U, mut buf: &[u8]) -> Result<(), RtuError> {
    while !buf.is_empty() {
        match uart.write(buf) {
            Ok(0) => return Err(RtuError::Io),
            Ok(n) => buf = &buf[n..],
            Err(_) => return Err(RtuError::Io),
        }
    }
    uart.flush().map_err(|_| RtuError::Io)?;
    Ok(())
}

/// Fill `buf` from the UART, treating "no bytes within
/// `response_timeout_ms`" as a timeout. Each `read` call gets a fresh
/// timeout — a slave that starts replying mid-window has the rest of
/// its frame protected by the next read's full budget. Worst-case
/// wall-clock is a small multiple of `response_timeout_ms` only if the
/// slave dribbles bytes with timeout-length pauses between them, which
/// no real Modbus device does.
fn read_exact<U: BlockingRead>(
    uart: &mut U,
    buf: &mut [u8],
    response_timeout_ms: u32,
) -> Result<(), RtuError> {
    let mut filled = 0;
    while filled < buf.len() {
        match uart.read(&mut buf[filled..], response_timeout_ms) {
            Ok(0) => return Err(RtuError::Timeout),
            Ok(n) => filled += n,
            Err(_) => return Err(RtuError::Io),
        }
    }
    Ok(())
}

/// Read a response of expected length `full_len`, short-circuiting on a
/// 5-byte Modbus exception frame.
fn read_response<'b, U: BlockingRead>(
    uart: &mut U,
    buf: &'b mut [u8],
    full_len: usize,
    response_timeout_ms: u32,
) -> Result<&'b [u8], RtuError> {
    assert!(full_len >= 5 && full_len <= buf.len());
    read_exact(uart, &mut buf[..3], response_timeout_ms)?;
    if buf[1] & EXCEPTION_BIT != 0 {
        read_exact(uart, &mut buf[3..5], response_timeout_ms)?;
        return Ok(&buf[..5]);
    }
    if full_len > 3 {
        read_exact(uart, &mut buf[3..full_len], response_timeout_ms)?;
    }
    Ok(&buf[..full_len])
}

// ─── ModbusTransport impl ────────────────────────────────────────────────────

impl<U, D> ModbusTransport for UartTransport<U, D>
where
    U: BlockingRead + Write,
    D: DelayNs,
{
    fn read_holding(&mut self, slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError> {
        assert!(slave != 0, "read does not support broadcast");
        assert!(!dst.is_empty() && dst.len() <= MAX_READ_REGS);
        let count = dst.len() as u16;
        let req = build_read_request(slave, addr, count);
        let expected_len = 5 + 2 * dst.len();

        self.pre_tx_silence();
        write_all(&mut self.uart, &req)?;

        let resp = read_response(
            &mut self.uart,
            &mut self.buf,
            expected_len,
            self.response_timeout_ms,
        )?;
        parse_read_response(resp, slave, dst)?;
        Ok(())
    }

    fn write_single_holding(&mut self, slave: u8, addr: u16, value: u16) -> Result<(), RtuError> {
        assert!(
            slave != 0,
            "single-register write does not support broadcast"
        );
        let req = build_write_single_request(slave, addr, value);

        self.pre_tx_silence();
        write_all(&mut self.uart, &req)?;

        let resp = read_response(&mut self.uart, &mut self.buf, 8, self.response_timeout_ms)?;
        parse_write_single_response(resp, &req)?;
        Ok(())
    }

    fn write_multiple_holdings(
        &mut self,
        slave: u8,
        addr: u16,
        values: &[u16],
    ) -> Result<(), RtuError> {
        assert!(
            slave != 0,
            "multi-register write does not support broadcast"
        );
        assert!(!values.is_empty() && values.len() <= MAX_WRITE_REGS);

        // Build request into self.buf, send it, then reuse the same
        // buffer for the response — sequential, no aliasing.
        let n = build_write_multiple_request(slave, addr, values, &mut self.buf)
            .expect("inputs validated above");

        self.pre_tx_silence();
        write_all(&mut self.uart, &self.buf[..n])?;

        let resp = read_response(&mut self.uart, &mut self.buf, 8, self.response_timeout_ms)?;
        parse_write_multiple_response(resp, slave, addr, values.len() as u16)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
