//! Driver for the XY7025 programmable buck converter. Other XY-series devices
//! remain accessible through the raw protocol layers.
//!
//! These modules share a common Modbus-RTU register layout — see the
//! crate's `DATASHEET.md` for the full protocol reference.
//!
//! ```no_run
//! # use xy_modbus::{RtuError, XyError};
//! # use xy_modbus::transport::ModbusTransport;
//! # struct MyTransport;
//! # impl ModbusTransport for MyTransport {
//! #     fn read_holding(&mut self, _: u8, _: u16, _: &mut [u16]) -> Result<(), RtuError> { unimplemented!() }
//! #     fn write_single_holding(&mut self, _: u8, _: u16, _: u16) -> Result<(), RtuError> { unimplemented!() }
//! #     fn write_multiple_holdings(&mut self, _: u8, _: u16, _: &[u16]) -> Result<(), RtuError> { unimplemented!() }
//! # }
//! # fn main() -> Result<(), XyError> {
//! # let my_transport = MyTransport;
//! use xy_modbus::{SafetyLimits, Xy};
//!
//! let mut xy = Xy::new(my_transport);
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
//! `uart` module provides a ready-to-use [`transport::ModbusTransport`] over any
//! `embedded-io` UART. To use a different transport, disable default
//! features and implement [`transport::ModbusTransport`] yourself; the [`framing`]
//! module exposes the on-wire codec.
//!
//! The [`regs`] module publishes the register map the raw accessors
//! ([`Xy::read_raw_holding`], [`Xy::write_raw_holding`]) take, so callers
//! reaching past the typed API need not hard-code addresses.
//!
//! For `esp-idf-hal` users, the `esp-idf-hal` feature ships a convenience
//! constructor so you don't need to write a UART wrapper:
//!
//! ```ignore
//! use xy_modbus::Xy;
//!
//! let mut xy = Xy::from_esp_uart(uart);
//! xy.set_voltage(13.5)?;
//! ```

#![no_std]

mod device;
mod error;
mod types;

pub mod framing;
pub mod regs;
pub mod transport;

#[cfg(feature = "embedded-io")]
pub mod uart;

// `esp-idf-hal` itself is target-conditional (only present when targeting
// `target_os = "espidf"`), so this module is too — enabling the feature on
// host builds is harmless.
#[cfg(all(feature = "esp-idf-hal", target_os = "espidf"))]
pub mod esp_idf;

pub use device::{ScaleCheck, Xy};
pub use error::{InputError, InputField, IoErrorKind, IoOperation, RtuError, XyError};
pub use types::enums::{BaudRate, ProtectionStatus, RegMode, TempUnit};
pub use types::group::GroupParams;
pub use types::status::{OnTime, SafetyLimits, Setpoints, Status, Temperature, Totals};
