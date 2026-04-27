//! Driver for the XY7025 programmable buck converter (and protocol-compatible
//! siblings via `Model::Custom`).
//!
//! These modules share a common Modbus-RTU register layout — see the
//! crate's `README.md` for the full protocol reference.
//!
//! ```no_run
//! # use xy_modbus::{ModbusTransport, RtuError};
//! # struct MyTransport;
//! # impl ModbusTransport for MyTransport {
//! #     fn read_holding(&mut self, _: u8, _: u16, _: &mut [u16]) -> Result<(), RtuError> { unimplemented!() }
//! #     fn write_single_holding(&mut self, _: u8, _: u16, _: u16) -> Result<(), RtuError> { unimplemented!() }
//! #     fn write_multiple_holdings(&mut self, _: u8, _: u16, _: &[u16]) -> Result<(), RtuError> { unimplemented!() }
//! # }
//! # fn main() -> Result<(), RtuError> {
//! # let my_transport = MyTransport;
//! use xy_modbus::{Model, Xy, SafetyLimits};
//!
//! let mut xy = Xy::new(my_transport, Model::Xy7025);
//! xy.set_protection(SafetyLimits { lvp_v: 22.0, ovp_v: 15.0, ocp_a: 15.0 })?;
//! xy.set_voltage(13.5)?;
//! xy.set_current_limit(10.0)?;
//! xy.set_output(true)?;
//!
//! let s = xy.read_status()?;
//! println!("{:.2} V @ {:.2} A", s.v_out, s.i_out);
//! # Ok(())
//! # }
//! ```
//!
//! The crate is `no_std`. With the default `embedded-io` feature, the
//! [`uart`] module provides a ready-to-use [`ModbusTransport`] over any
//! `embedded-io` UART. To use a different transport, disable default
//! features and implement [`ModbusTransport`] yourself; the [`framing`]
//! module exposes the on-wire codec.
//!
//! For `esp-idf-hal` users, the `esp-idf-hal` feature ships a convenience
//! constructor so you don't need to write a UART wrapper:
//!
//! ```ignore
//! use xy_modbus::{Model, Xy};
//!
//! let mut xy = Xy::from_esp_uart(uart, Model::Xy7025);
//! xy.set_voltage(13.5)?;
//! ```

#![no_std]

// ─── Modules ─────────────────────────────────────────────────────────────────

mod device;
mod types;

pub mod framing;
pub(crate) mod regs;
pub mod transport;

#[cfg(feature = "embedded-io")]
pub mod uart;

// `esp-idf-hal` itself is target-conditional (only present when targeting
// `target_os = "espidf"`), so this module is too — enabling the feature on
// host builds is harmless.
#[cfg(all(feature = "esp-idf-hal", target_os = "espidf"))]
pub mod esp_idf;

// ─── Re-exports ──────────────────────────────────────────────────────────────

pub use device::Xy;
pub use framing::FrameError;
pub use transport::{BlockingRead, ModbusError, ModbusTransport, RtuError};
pub use types::{
    BaudRate, GroupParams, Model, ModelCheck, OnTime, ProtectionStatus, RegMode, SafetyLimits,
    Setpoints, Status, TempUnit, Temperatures, Totals,
};

#[cfg(feature = "embedded-io")]
pub use uart::UartTransport;
