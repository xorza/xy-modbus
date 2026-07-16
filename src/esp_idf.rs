//! Convenience glue for `esp-idf-hal` UART drivers.
//!
//! `UartDriver` already implements `embedded_io::Write` and exposes a
//! native `read(buf, ticks)`; impl-ing [`BlockingRead`] on it is a
//! one-liner. The constructor below ties it to the bundled
//! [`UartTransport`] so callers don't write any glue:
//!
//! ```ignore
//! use xy_modbus::Xy;
//!
//! let mut xy = Xy::from_esp_uart(uart);
//! xy.set_protection(safety)?;
//! xy.set_voltage(13.5)?;
//! xy.set_output(true)?;
//! ```

use core::time::Duration;

use esp_idf_hal::delay::{FreeRtos, TickType};
use esp_idf_hal::io::EspIOError;
use esp_idf_hal::uart::UartDriver;

use crate::device::Xy;
use crate::uart::{BlockingRead, UartTransport};

impl BlockingRead for UartDriver<'_> {
    fn read(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, Self::Error> {
        let ticks = TickType::from(Duration::from_millis(timeout_ms as u64)).ticks();
        UartDriver::read(self, buf, ticks).map_err(EspIOError)
    }
}

/// Concrete transport type produced by [`Xy::from_esp_uart`].
pub type EspIdfTransport<'d> = UartTransport<UartDriver<'d>, FreeRtos>;

impl<'d> Xy<EspIdfTransport<'d>> {
    /// Wrap an `esp_idf_hal::uart::UartDriver` with the default XY-series
    /// timing (500 ms response window, 50 ms inter-frame gap). For
    /// non-default timing, build the transport manually:
    ///
    /// ```ignore
    /// use xy_modbus::uart::{UartTiming, UartTransport};
    ///
    /// let timing = UartTiming::new(750, 100, 10).unwrap();
    /// let transport = UartTransport::new(uart, FreeRtos).with_timing(timing);
    /// let xy = Xy::new(transport);
    /// ```
    pub fn from_esp_uart(uart: UartDriver<'d>) -> Self {
        Self::new(UartTransport::new(uart, FreeRtos))
    }
}
