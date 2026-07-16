//! Default Modbus-RTU transport over a [`BlockingRead`] +
//! [`embedded_io::Write`] UART.
//!
//! Wrap any UART driver that exposes a blocking-with-timeout read (every
//! kernel-backed HAL does) plus an [`embedded_hal::delay::DelayNs`] timer
//! for the inter-frame silence, and you have a working
//! [`crate::transport::ModbusTransport`].
//!
//! ```ignore
//! use xy_modbus::{uart::UartTransport, Xy};
//!
//! let transport = UartTransport::new(uart, delay);
//! let mut xy = Xy::new(transport);
//! ```
//!
//! Timing defaults use a 500 ms per-read inactivity timeout, a 50 ms
//! inter-frame gap, and up to ten quiet-bus acquisition attempts. Override
//! them with [`UartTransport::with_timing`] and [`UartTiming`].
//! Write requests to slave `0` use standard Modbus broadcast semantics: the
//! frame is flushed and the call returns without waiting for a response.

use embedded_hal::delay::DelayNs;
use embedded_io::Write;

use crate::framing::{self, MAX_ADU, MAX_READ_REGS, MAX_WRITE_REGS, ResponseShape};
use crate::transport::{IoErrorKind, IoOperation, ModbusTransport, RtuError};

// `write_multiple_holdings` builds the request frame into `self.buf` and
// `expect`s success — sound only because the buffer fits the largest
// possible Write Multiple frame (slave + fc + addr + qty + bc + 2*regs + crc).
const _: () = assert!(MAX_ADU >= 9 + 2 * MAX_WRITE_REGS);

/// Read side of [`UartTransport`].
///
/// Wraps a UART driver that already knows how to block efficiently for
/// incoming bytes. Avoiding `embedded_io::ReadReady` is deliberate: many
/// kernel-backed HALs do not implement it and already block cheaply.
pub trait BlockingRead: embedded_io::ErrorType {
    /// Block for up to `timeout_ms` waiting for at least one byte, then
    /// return up to `buf.len()` bytes. `Ok(0)` means no byte arrived.
    /// A zero timeout is a non-blocking poll used for the pre-TX flush.
    fn read(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, Self::Error>;
}

/// Validated UART transaction timing.
///
/// `quiet_attempts` bounds how many complete inter-frame gaps may still end in
/// RX activity before a transaction returns [`RtuError::BusBusy`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UartTiming {
    read_timeout_ms: u32,
    inter_frame_ms: u32,
    quiet_attempts: u32,
}

impl UartTiming {
    /// XY-series timing used by [`UartTransport::new`].
    pub const DEFAULT: Self = Self {
        read_timeout_ms: 500,
        inter_frame_ms: 50,
        quiet_attempts: 10,
    };

    /// Build a timing configuration whose values are all nonzero.
    ///
    /// `read_timeout_ms` is the inactivity timeout for each partial response
    /// read. `inter_frame_ms` is the required quiet interval before a request.
    /// `quiet_attempts` bounds how many such intervals may end in RX activity.
    pub const fn new(
        read_timeout_ms: u32,
        inter_frame_ms: u32,
        quiet_attempts: u32,
    ) -> Result<Self, UartTimingError> {
        if read_timeout_ms == 0 {
            return Err(UartTimingError::ZeroReadTimeout);
        }
        if inter_frame_ms == 0 {
            return Err(UartTimingError::ZeroInterFrameGap);
        }
        if quiet_attempts == 0 {
            return Err(UartTimingError::ZeroQuietAttempts);
        }
        Ok(Self {
            read_timeout_ms,
            inter_frame_ms,
            quiet_attempts,
        })
    }
}

impl Default for UartTiming {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Invalid [`UartTiming`] parameter.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartTimingError {
    /// The per-read response timeout was zero.
    ZeroReadTimeout,
    /// The required pre-transmit quiet interval was zero.
    ZeroInterFrameGap,
    /// The quiet-bus acquisition attempt budget was zero.
    ZeroQuietAttempts,
}

impl core::fmt::Display for UartTimingError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::ZeroReadTimeout => "UART read timeout must be nonzero",
            Self::ZeroInterFrameGap => "UART inter-frame gap must be nonzero",
            Self::ZeroQuietAttempts => "UART quiet-attempt count must be nonzero",
        })
    }
}

impl core::error::Error for UartTimingError {}

/// Values recovered from [`UartTransport::into_parts`].
#[derive(Debug)]
pub struct UartParts<U, D> {
    pub uart: U,
    pub delay: D,
}

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
    timing: UartTiming,
    buf: [u8; MAX_ADU],
}

impl<U, D> UartTransport<U, D>
where
    U: BlockingRead + Write,
    D: DelayNs,
{
    /// Build a transport with a 500 ms per-read inactivity timeout, a 50 ms
    /// inter-frame quiet gap, and ten quiet-bus acquisition attempts.
    pub fn new(uart: U, delay: D) -> Self {
        Self {
            uart,
            delay,
            timing: UartTiming::DEFAULT,
            buf: [0u8; MAX_ADU],
        }
    }

    /// Override the transaction timing with a validated configuration.
    pub fn with_timing(mut self, timing: UartTiming) -> Self {
        self.timing = timing;
        self
    }

    /// Recover the inner UART and delay.
    pub fn into_parts(self) -> UartParts<U, D> {
        UartParts {
            uart: self.uart,
            delay: self.delay,
        }
    }

    /// Enforce ≥t3.5 bus silence before the next master frame.
    fn pre_tx_silence(&mut self) -> Result<(), RtuError> {
        for _ in 0..self.timing.quiet_attempts {
            self.delay.delay_ms(self.timing.inter_frame_ms);
            if drain_rx(&mut self.uart)? == DrainOutcome::Quiet {
                return Ok(());
            }
        }
        Err(RtuError::BusBusy)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum DrainOutcome {
    Quiet,
    Activity,
}

/// Discard whatever is already in the UART RX buffer. Cheap
/// non-blocking calls (`timeout_ms = 0`) drain noise that arrived during
/// the inter-frame silence so it doesn't masquerade as the start of the
/// slave's reply.
///
/// Each pass is capped at `DRAIN_MAX_BYTES` so a driver that continuously
/// returns data cannot spin here indefinitely. Reaching the cap reports
/// activity and makes the caller wait another complete quiet interval.
fn drain_rx<U: BlockingRead>(uart: &mut U) -> Result<DrainOutcome, RtuError> {
    const DRAIN_MAX_BYTES: usize = 4 * MAX_ADU;
    let mut scratch = [0u8; 32];
    let mut drained = 0;
    while drained < DRAIN_MAX_BYTES {
        match uart.read(&mut scratch, 0) {
            Ok(0) if drained == 0 => return Ok(DrainOutcome::Quiet),
            Ok(0) => return Ok(DrainOutcome::Activity),
            Ok(n) => drained += n,
            Err(error) if is_interrupted(&error) => continue,
            Err(error) => return Err(io_error(IoOperation::Read, error)),
        }
    }
    Ok(DrainOutcome::Activity)
}

fn io_error<E: embedded_io::Error>(operation: IoOperation, error: E) -> RtuError {
    RtuError::Io {
        operation,
        kind: IoErrorKind::from(error.kind()),
    }
}

fn is_interrupted<E: embedded_io::Error>(error: &E) -> bool {
    error.kind() == embedded_io::ErrorKind::Interrupted
}

fn validate_read_slave(slave: u8) -> Result<(), RtuError> {
    if (1..=247).contains(&slave) {
        Ok(())
    } else {
        Err(RtuError::InvalidSlaveAddress(slave))
    }
}

fn validate_write_slave(slave: u8) -> Result<(), RtuError> {
    if slave <= 247 {
        Ok(())
    } else {
        Err(RtuError::InvalidSlaveAddress(slave))
    }
}

fn write_all<U: Write>(uart: &mut U, mut buf: &[u8]) -> Result<(), RtuError> {
    while !buf.is_empty() {
        match uart.write(buf) {
            Ok(0) => {
                return Err(RtuError::Io {
                    operation: IoOperation::Write,
                    kind: IoErrorKind::WriteZero,
                });
            }
            Ok(n) => buf = &buf[n..],
            Err(error) if is_interrupted(&error) => continue,
            Err(error) => return Err(io_error(IoOperation::Write, error)),
        }
    }
    loop {
        match uart.flush() {
            Ok(()) => return Ok(()),
            Err(error) if is_interrupted(&error) => continue,
            Err(error) => return Err(io_error(IoOperation::Flush, error)),
        }
    }
}

/// Fill `buf` from the UART, treating "no bytes within
/// `read_timeout_ms`" as a timeout. Each `read` call gets a fresh
/// timeout. This currently acts as an inactivity timeout rather than an
/// overall transaction deadline.
fn read_exact<U: BlockingRead>(
    uart: &mut U,
    buf: &mut [u8],
    read_timeout_ms: u32,
) -> Result<(), RtuError> {
    let mut filled = 0;
    while filled < buf.len() {
        match uart.read(&mut buf[filled..], read_timeout_ms) {
            Ok(0) => return Err(RtuError::Timeout),
            Ok(n) => filled += n,
            Err(error) if is_interrupted(&error) => continue,
            Err(error) => return Err(io_error(IoOperation::Read, error)),
        }
    }
    Ok(())
}

/// Read and classify a response prefix before acquiring the remaining bytes.
fn read_response<'b, U: BlockingRead>(
    uart: &mut U,
    buf: &'b mut [u8],
    slave: u8,
    shape: ResponseShape,
    read_timeout_ms: u32,
) -> Result<&'b [u8], RtuError> {
    read_exact(uart, &mut buf[..3], read_timeout_ms)?;
    let full_len = framing::response_adu_len([buf[0], buf[1], buf[2]], slave, shape)?;
    debug_assert!((5..=buf.len()).contains(&full_len));
    read_exact(uart, &mut buf[3..full_len], read_timeout_ms)?;
    Ok(&buf[..full_len])
}

impl<U, D> ModbusTransport for UartTransport<U, D>
where
    U: BlockingRead + Write,
    D: DelayNs,
{
    fn read_holding(&mut self, slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError> {
        validate_read_slave(slave)?;
        if dst.is_empty() || dst.len() > MAX_READ_REGS {
            return Err(RtuError::InvalidQuantity(dst.len()));
        }
        let count = dst.len() as u16;
        let req = framing::build_read_request(slave, addr, count)
            .expect("slave and register quantity validated above");

        self.pre_tx_silence()?;
        write_all(&mut self.uart, &req)?;

        let resp = read_response(
            &mut self.uart,
            &mut self.buf,
            slave,
            ResponseShape::ReadHolding {
                register_count: dst.len(),
            },
            self.timing.read_timeout_ms,
        )?;
        framing::parse_read_response(resp, slave, dst)?;
        Ok(())
    }

    fn write_single_holding(&mut self, slave: u8, addr: u16, value: u16) -> Result<(), RtuError> {
        validate_write_slave(slave)?;
        let req = framing::build_write_single_request(slave, addr, value)
            .expect("slave address validated above");

        self.pre_tx_silence()?;
        write_all(&mut self.uart, &req)?;
        if slave == 0 {
            return Ok(());
        }

        let resp = read_response(
            &mut self.uart,
            &mut self.buf,
            slave,
            ResponseShape::WriteSingle,
            self.timing.read_timeout_ms,
        )?;
        framing::parse_write_single_response(resp, &req)?;
        Ok(())
    }

    fn write_multiple_holdings(
        &mut self,
        slave: u8,
        addr: u16,
        values: &[u16],
    ) -> Result<(), RtuError> {
        validate_write_slave(slave)?;
        if values.is_empty() || values.len() > MAX_WRITE_REGS {
            return Err(RtuError::InvalidQuantity(values.len()));
        }

        // Build request into self.buf, send it, then reuse the same
        // buffer for the response — sequential, no aliasing.
        let n = framing::build_write_multiple_request(slave, addr, values, &mut self.buf)
            .expect("inputs validated above");

        self.pre_tx_silence()?;
        write_all(&mut self.uart, &self.buf[..n])?;
        if slave == 0 {
            return Ok(());
        }

        let resp = read_response(
            &mut self.uart,
            &mut self.buf,
            slave,
            ResponseShape::WriteMultiple,
            self.timing.read_timeout_ms,
        )?;
        framing::parse_write_multiple_response(resp, slave, addr, values.len() as u16)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
